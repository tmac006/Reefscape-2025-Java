package org.team1507.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

import org.team1507.lib.logging.phoenix.Pigeon2Logger;
import org.team1507.lib.swerve.SwerveAPI;
import org.team1507.lib.swerve.config.SwerveConfig;
import org.team1507.lib.swerve.hardware.SwerveIMUs.SwerveIMU.IMUSimHook;
import org.team1507.lib.util.Mutable;
import org.team1507.lib.util.vendors.PhoenixUtil;

/**
 * Contains implementations for IMUs to be used with the {@link SwerveAPI}.
 */
public final class SwerveIMUs {

    private SwerveIMUs() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * A swerve IMU.
     */
    public abstract static class SwerveIMU implements SwerveBaseHardware {

        /**
         * {@code (config) -> SwerveIMU}
         */
        @FunctionalInterface
        public static interface Ctor extends Function<SwerveConfig, SwerveIMU> {}

        /**
         * Provides consumer to be saved and invoked periodically with the
         * robot's current chassis speeds to update the simulated IMU.
         */
        @FunctionalInterface
        public static interface IMUSimHook extends Consumer<Consumer<ChassisSpeeds>> {}

        /**
         * Constructs a swerve IMU. Wraps to support simulation if applicable.
         * @param ctor The IMU's constructor.
         * @param config The general swerve API configuration.
         * @param simHook Hook to update the IMU if simulation is active.
         */
        public static SwerveIMU construct(Ctor ctor, SwerveConfig config, IMUSimHook simHook) {
            SwerveIMU imu = ctor.apply(config);
            if (RobotBase.isSimulation()) imu = simulate(imu, config, simHook);
            return imu;
        }

        /**
         * Gets the IMU's absolute yaw.
         */
        public abstract Rotation2d getYaw();

        /**
         * Gets the IMU's absolute pitch.
         */
        public abstract Rotation2d getPitch();

        /**
         * Gets the IMU's absolute roll.
         */
        public abstract Rotation2d getRoll();
    }

    /**
     * Configures a {@link Pigeon2}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     */
    public static SwerveIMU.Ctor pigeon2(int id) {
        return config -> {
            var deviceLogger = new Pigeon2Logger();
            Pigeon2 pigeon2 = new Pigeon2(id, config.phoenixCanBus);

            StatusSignal<Angle> yaw = pigeon2.getYaw().clone();
            StatusSignal<Angle> pitch = pigeon2.getPitch();
            StatusSignal<Angle> roll = pigeon2.getRoll();
            StatusSignal<AngularVelocity> yawVelocity = pigeon2.getAngularVelocityZWorld().clone();
            StatusSignal<AngularVelocity> pitchVelocity = pigeon2.getAngularVelocityXWorld();
            StatusSignal<AngularVelocity> rollVelocity = pigeon2.getAngularVelocityYWorld();

            PhoenixUtil.run("Clear Sticky Faults", () -> pigeon2.clearStickyFaults());
            PhoenixUtil.run("Set Update Frequency", () ->
                BaseStatusSignal.setUpdateFrequencyForAll(
                    1.0 / config.odometryPeriod,
                    yaw,
                    pitch,
                    roll,
                    yawVelocity,
                    pitchVelocity,
                    rollVelocity
                )
            );
            PhoenixUtil.run("Optimize Bus Utilization", () ->
                pigeon2.optimizeBusUtilization(1.0 / config.defaultFramePeriod, 0.05)
            );

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity)
                    );
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(pitch.refresh(), pitchVelocity.refresh())
                    );
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(roll.refresh(), rollVelocity.refresh())
                    );
                }

                @Override
                public Object getAPI() {
                    return pigeon2;
                }

                @Override
                public void log(EpilogueBackend backend, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(backend, pigeon2, errorHandler);
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(yaw, yawVelocity);
                }

                @Override
                public void close() {
                    pigeon2.close();
                }
            };
        };
    }

    /**
     * Rudimentary IMU simulation wrapper. Calculates yaw based on the robot's angular velocity.
     * @param imu The IMU to wrap.
     * @param config The general swerve API configuration.
     * @param simHook Hook to update the IMU if simulation is active.
     */
    private static SwerveIMU simulate(SwerveIMU imu, SwerveConfig config, IMUSimHook simHook) {
        Mutable<Double> yaw = new Mutable<>(0.0);
        simHook.accept(speeds -> {
            yaw.value += speeds.omegaRadiansPerSecond * config.period;
        });

        return new SwerveIMU() {
            @Override
            public Rotation2d getYaw() {
                return Rotation2d.fromRadians(yaw.value);
            }

            @Override
            public Rotation2d getPitch() {
                return Rotation2d.kZero;
            }

            @Override
            public Rotation2d getRoll() {
                return Rotation2d.kZero;
            }

            @Override
            public Object getAPI() {
                return imu.getAPI();
            }

            @Override
            public void log(EpilogueBackend backend, ErrorHandler errorHandler) {
                imu.log(backend, errorHandler);
                var sim = backend.getNested(".sim");
                sim.log("yaw", getYaw(), Rotation2d.struct);
                sim.log("pitch", getPitch(), Rotation2d.struct);
                sim.log("roll", getRoll(), Rotation2d.struct);
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return imu.getSignals();
            }

            @Override
            public boolean readError() {
                return imu.readError();
            }

            @Override
            public void close() {
                try {
                    imu.close();
                } catch (Exception e) {}
            }
        };
    }
}
