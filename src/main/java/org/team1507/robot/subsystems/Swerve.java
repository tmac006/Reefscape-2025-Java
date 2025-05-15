package org.team1507.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team1507.lib.math.Math2;
import org.team1507.lib.math.PAPFController;
import org.team1507.lib.swerve.Perspective;
import org.team1507.lib.swerve.SwerveAPI;
import org.team1507.lib.swerve.SwerveState;
import org.team1507.lib.swerve.config.SwerveConfig;
import org.team1507.lib.swerve.config.SwerveModuleConfig;
import org.team1507.lib.swerve.hardware.SwerveEncoders;
import org.team1507.lib.swerve.hardware.SwerveIMUs;
import org.team1507.lib.swerve.hardware.SwerveMotors;
import org.team1507.lib.util.Alliance;
import org.team1507.lib.util.Mutable;
import org.team1507.lib.util.Tunable;
import org.team1507.lib.util.Tunable.TunableDouble;
import org.team1507.lib.util.command.GRRSubsystem;
import org.team1507.robot.Constants;
import org.team1507.robot.Constants.LowerCAN;
import org.team1507.robot.Constants.RioCAN;
import org.team1507.robot.util.Field;
import org.team1507.robot.util.Vision;
import org.team1507.robot.util.Field.ReefLocation;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final TunableDouble TURBO_SPIN = Tunable.doubleValue("swerve/turboSpin", 8.0);
    private static final TunableDouble IN_LINE_TOLERANCE = Tunable.doubleValue("swerve/inLineTolerance", 0.35);

    private static final TunableDouble BEACH_SPEED = Tunable.doubleValue("swerve/beach/speed", 3.0);
    private static final TunableDouble BEACH_TOLERANCE = Tunable.doubleValue("swerve/beach/tolerance", 0.15);

    private static final TunableDouble APF_X = Tunable.doubleValue("swerve/apf/x", 1.05);
    private static final TunableDouble APF_VEL = Tunable.doubleValue("swerve/apf/velocity", 4.5);
    private static final TunableDouble APF_LEAD = Tunable.doubleValue("swerve/apf/lead", 0.78);
    private static final TunableDouble APF_LEAD_MULT = Tunable.doubleValue("swerve/apf/leadMult", 0.35);
    private static final TunableDouble APF_LEAD_ACCEL = Tunable.doubleValue("swerve/apf/leadAccel", 7.7);
    private static final TunableDouble APF_LEAD_ACCEL_L4 = Tunable.doubleValue("swerve/apf/leadAccelL4", 5.78);
    private static final TunableDouble APF_SCORE_ACCEL = Tunable.doubleValue("swerve/apf/scoreAccel", 4.8);
    private static final TunableDouble APF_TOLERANCE = Tunable.doubleValue("swerve/apf/tolerance", 0.2);
    private static final TunableDouble APF_ANG_TOLERANCE = Tunable.doubleValue("swerve/apf/angTolerance", 0.4);

    private static final TunableDouble REEF_ASSIST_X = Tunable.doubleValue("swerve/reefAssist/x", 0.681);
    private static final TunableDouble REEF_ASSIST_KP = Tunable.doubleValue("swerve/reefAssist/kP", 20.0);
    private static final TunableDouble REEF_ASSIST_TOLERANCE = Tunable.doubleValue("swerve/reefAssist/tolerance", 1.75);

    private static final TunableDouble FACING_REEF_TOLERANCE = Tunable.doubleValue("swerve/kFacingReefTolerance", 1.0);
    private static final TunableDouble DANGER_DISTANCE = Tunable.doubleValue("swerve/dangerDistance", 0.7);
    private static final TunableDouble HAPPY_DISTANCE = Tunable.doubleValue("swerve/happyDistance", 3.25);
    private static final TunableDouble GOOSING_DISTANCE = Tunable.doubleValue("swerve/goosingDistance", 0.95);

    private static final double OFFSET = Units.inchesToMeters(12.5);

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FL_ENCODER, 0.296, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FR_ENCODER, -0.395, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.BL_ENCODER, 0.190, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.BR_ENCODER, -0.079, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02, 0.01)
        .setMovePID(0.3, 0.0, 0.0)
        .setMoveFF(0.15, 0.128)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.5, 0.05, 16.5, 11.5, 28.0)
        .setDriverProfile(4.5, 1.5, 0.15, 4.7, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(243.0 / 38.0, 12.1, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.pigeon2( 0 ))
        .setPhoenixFeatures(new CANBus(LowerCAN.LOWER_CAN), true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    private final SwerveAPI api;
    private final Vision vision;
    private final SwerveState state;
    private final PAPFController apf;

    private final ProfiledPIDController angularPID;

    private final ReefAssistData reefAssist = new ReefAssistData();

    private Pose2d reefReference = Pose2d.kZero;
    private boolean facingReef = false;
    private double wallDistance = 0.0;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(Constants.CAMERAS);
        state = api.state;
        apf = new PAPFController(4.0, 0.5, true, Field.OBSTACLES);

        angularPID = new ProfiledPIDController(10.0, 0.0, 0.0, new Constraints(10.0, 24.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        api.enableTunables("swerve/api");
        apf.enableTunables("swerve/apf");
        Tunable.pidController("swerve/angularPID", angularPID);
    }

    @Override
    public void periodic() {
        // Refresh the swerve API.
        api.refresh();

        // Apply vision estimates to the pose estimator.
        api.addVisionMeasurements(vision.getUnreadResults(state.poseHistory, state.odometryPose));

        // Calculate helpers
        Translation2d reefCenter = Alliance.isBlue() ? Field.REEF_BLUE : Field.REEF_RED;
        Translation2d reefTranslation = state.translation.minus(reefCenter);
        Rotation2d reefAngle = new Rotation2d(
            Math.floor(
                reefCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math.PI / 6.0)).getRadians() /
                (Math.PI / 3.0)
            ) *
            (Math.PI / 3.0)
        );

        // Save the current alliance's reef location, and the rotation
        // to the reef wall relevant to the robot's position.
        reefReference = new Pose2d(reefCenter, reefAngle);

        // If the robot is rotated to face the reef, within an arbitrary tolerance.
        facingReef = Math2.isNear(reefAngle, state.rotation, FACING_REEF_TOLERANCE.value());

        // Calculate the distance from the robot's center to the nearest reef wall face.
        wallDistance = Math.max(
            0.0,
            reefAngle.rotateBy(Rotation2d.kPi).minus(reefTranslation.getAngle()).getCos() * reefTranslation.getNorm() -
            Field.REEF_WALL_DIST
        );
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
    }

    /**
     * Returns the directionless measured velocity of the robot, in m/s.
     */
    @NotLogged
    public double getVelocity() {
        return state.velocity;
    }

    /**
     * Remove @NotLogged for debugging
     */
    // @NotLogged
    public List<Pose2d> apfVisualization() {
        return apf.visualizeField(30, 1.0, Field.LENGTH, Field.WIDTH);
    }

    /**
     * Returns true if the goose is happy!!
     * (Robot is facing the reef and within the happy distance).
     */
    public boolean happyGoose() {
        return (
            facingReef &&
            wallDistance < HAPPY_DISTANCE.value() &&
            Math.abs(reefAssist.error) < IN_LINE_TOLERANCE.value()
        );
    }

    /**
     * Returns true if the elevator and goose neck are safe
     * to move, based on the robot's position on the field.
     */
    public boolean wildlifeConservationProgram() {
        return wallDistance > DANGER_DISTANCE.value();
    }

    public boolean goosingTime() {
        return wallDistance < GOOSING_DISTANCE.value();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> {
                api.tareRotation(Perspective.OPERATOR);
                vision.reset();
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin
     *             relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> {
                api.resetPose(pose.get());
                vision.reset();
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            double pitch = state.pitch.getRadians();
            double roll = state.roll.getRadians();

            var antiBeach = Perspective.OPERATOR.toPerspectiveSpeeds(
                new ChassisSpeeds(
                    Math.abs(pitch) > BEACH_TOLERANCE.value() ? Math.copySign(BEACH_SPEED.value(), pitch) : 0.0,
                    Math.abs(roll) > BEACH_TOLERANCE.value() ? Math.copySign(BEACH_SPEED.value(), -roll) : 0.0,
                    0.0
                ),
                state.rotation
            );

            api.applyAssistedDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                antiBeach,
                Perspective.OPERATOR,
                true,
                true
            );
        });
    }

    /**
     * SPIN FAST RAHHHHHHH
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command turboSpin(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        Mutable<Double> configured = new Mutable<>(0.0);
        return drive(x, y, angular)
            .beforeStarting(() -> {
                configured.value = api.config.driverAngularVel;
                api.config.driverAngularVel = TURBO_SPIN.value();
            })
            .finallyDo(() -> api.config.driverAngularVel = configured.value);
    }

    /**
     * Drives the robot using driver input while facing the reef,
     * and "pushing" the robot to center on the selected pipe.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     */
    public Command driveReef(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular, BooleanSupplier left) {
        Mutable<Boolean> exitLock = new Mutable<>(false);

        return commandBuilder("Swerve.driveReef()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);
                exitLock.value = false;
            })
            .onExecute(() -> {
                double xInput = x.getAsDouble();
                double yInput = y.getAsDouble();
                double angularInput = angular.getAsDouble();
                double norm = Math.hypot(-yInput, -xInput);
                boolean inDeadband = norm < api.config.driverVelDeadband;

                reefAssist.targetPipe = generateReefLocation(
                    REEF_ASSIST_X.value(),
                    reefReference.getRotation(),
                    left.getAsBoolean()
                );

                Rotation2d robotAngle = reefAssist.targetPipe.getTranslation().minus(state.translation).getAngle();
                Rotation2d xyAngle = !inDeadband
                    ? new Rotation2d(-yInput, -xInput).rotateBy(Alliance.isBlue() ? Rotation2d.kZero : Rotation2d.kPi)
                    : robotAngle;

                double stickDistance = Math.abs(
                    (Math.cos(xyAngle.getRadians()) * (reefAssist.targetPipe.getY() - state.pose.getY()) -
                        Math.sin(xyAngle.getRadians()) * (reefAssist.targetPipe.getX() - state.pose.getX()))
                );

                reefAssist.running =
                    Math.abs(stickDistance) < REEF_ASSIST_TOLERANCE.value() &&
                    Math2.isNear(robotAngle, xyAngle, Math.PI / 2.0) &&
                    !inDeadband;

                reefAssist.error = robotAngle.minus(reefReference.getRotation()).getRadians();
                reefAssist.output = reefAssist.running ? reefAssist.error * norm * norm * REEF_ASSIST_KP.value() : 0.0;

                var assist = Perspective.OPERATOR.toPerspectiveSpeeds(
                    new ChassisSpeeds(
                        0.0,
                        reefAssist.output,
                        !exitLock.value
                            ? angularPID.calculate(
                                state.rotation.getRadians(),
                                reefReference.getRotation().getRadians()
                            )
                            : 0.0
                    ),
                    reefReference.getRotation()
                );

                if (Math.abs(angularInput) > 1e-6) exitLock.value = true;
                api.applyAssistedDriverInput(xInput, yInput, angularInput, assist, Perspective.OPERATOR, true, true);
            })
            .onEnd(() -> reefAssist.running = false);
    }

    /**
     * Drives the robot to the reef autonomously. Targets
     * the side of the reef that the robot is closest to.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command apfDrive(BooleanSupplier left, BooleanSupplier ready, BooleanSupplier l4) {
        return apfDrive(() -> reefReference.getRotation(), left, ready, l4);
    }

    /**
     * Drives the robot to the reef autonomously.
     * @param location The reef location to drive to.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command apfDrive(ReefLocation location, BooleanSupplier ready, BooleanSupplier l4) {
        return apfDrive(
            () -> Alliance.isBlue() ? location.side : location.side.rotateBy(Rotation2d.kPi),
            () -> location.left,
            ready,
            l4
        );
    }

    /**
     * Internal function, converts reef side to APF drive controller.
     * @param side A supplier that returns the side of the reef to target.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    private Command apfDrive(
        Supplier<Rotation2d> side,
        BooleanSupplier left,
        BooleanSupplier ready,
        BooleanSupplier l4
    ) {
        Mutable<Pose2d> lastTarget = new Mutable<>(Pose2d.kZero);
        Mutable<Boolean> nowSafe = new Mutable<>(false);

        return apfDrive(
            () -> {
                Pose2d target =
                    reefAssist.targetPipe = generateReefLocation(APF_X.value(), side.get(), left.getAsBoolean());

                Translation2d error = target.getTranslation().minus(state.translation);
                Rotation2d robotAngle = error.getAngle();
                reefAssist.error = robotAngle.minus(side.get()).getRadians();

                if (!target.equals(lastTarget.value)) nowSafe.value = false;
                lastTarget.value = target;

                if (!nowSafe.value) {
                    target = generateReefLocation(
                        APF_X.value() +
                        APF_LEAD.value() +
                        (APF_LEAD_MULT.value() * (error.getNorm() - APF_LEAD.value())),
                        side.get(),
                        left.getAsBoolean()
                    );

                    if (
                        ready.getAsBoolean() &&
                        state.translation.getDistance(target.getTranslation()) *
                        (Math.abs(reefAssist.error) / Math.PI) <=
                        APF_TOLERANCE.value() &&
                        Math.abs(state.rotation.minus(target.getRotation()).getRadians()) <= APF_ANG_TOLERANCE.value()
                    ) {
                        nowSafe.value = true;
                    }
                }

                return target;
            },
            () ->
                !nowSafe.value
                    ? (l4.getAsBoolean() ? APF_LEAD_ACCEL_L4.value() : APF_LEAD_ACCEL.value())
                    : APF_SCORE_ACCEL.value()
        ).beforeStarting(() -> {
            lastTarget.value = Pose2d.kZero;
            nowSafe.value = false;
        });
    }

    /**
     * Drives the robot to a target position using the APF, ending
     * when the robot is within a specified tolerance of the target.
     * @param target A supplier that returns the target blue-origin relative field location.
     * @param deceleration A supplier that returns the deceleration for the robot to target in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> target, DoubleSupplier deceleration, DoubleSupplier endTolerance) {
        return apfDrive(target, deceleration).until(
            () -> target.get().getTranslation().getDistance(state.translation) < endTolerance.getAsDouble()
        );
    }

    /**
     * Drives the robot to a target position using the APF.
     * @param target A supplier that returns the target blue-origin relative field location.
     * @param deceleration A supplier that returns the deceleration for the robot to target in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> target, DoubleSupplier deceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d goal = target.get();
                var speeds = apf.calculate(
                    state.pose,
                    goal.getTranslation(),
                    APF_VEL.value(),
                    deceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    goal.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    private Pose2d generateReefLocation(double xOffset, Rotation2d side, boolean left) {
        return new Pose2d(
            reefReference
                .getTranslation()
                .plus(new Translation2d(-xOffset, Field.PIPE_Y * (left ? 1.0 : -1.0)).rotateBy(side)),
            side
        );
    }

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }
}
