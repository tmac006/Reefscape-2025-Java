package org.team1507.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team1507.lib.util.Mutable;
import org.team1507.lib.util.Tunable;
import org.team1507.lib.util.Tunable.TunableDouble;
import org.team1507.lib.util.command.GRRSubsystem;
import org.team1507.lib.util.vendors.PhoenixUtil;
import org.team1507.robot.Constants.LowerCAN;
import org.team1507.robot.util.ReefSelection;

@Logged
public final class Elevator extends GRRSubsystem {

    public static enum ElevatorPosition {
        DOWN(0.0),
        INTAKE(0.18),
        L2(10.75),
        L3(22.5),
        L4(40.25);

        private final TunableDouble radians;
       

        private ElevatorPosition(double radians) {
            this(radians, false);
        }

        private ElevatorPosition(double radians, boolean scoring) {
            this.radians = Tunable.doubleValue("elevator/positions/" + name(), radians);
        }

        public double radians() {
            return radians.value();
        }

        private static ElevatorPosition closest(double position) {
            ElevatorPosition closest = null;
            double min = Double.MAX_VALUE;
            for (ElevatorPosition option : values()) {
                double distance = Math.abs(option.radians() - position);
                if (distance < CLOSEST_TOLERANCE.value() && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }

    private static final TunableDouble CLOSEST_TOLERANCE = Tunable.doubleValue("elevator/closestTolerance", 0.15);
    private static final TunableDouble AT_POSITION_TOLERANCE = Tunable.doubleValue("elevator/atPositionTolerance", 0.1);
    private static final TunableDouble ZERO_TOLERANCE = Tunable.doubleValue("elevator/zeroTolerance", 0.15);
    private final TalonFX leadMotor;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;



    private final MotionMagicVoltage positionControl;

    private boolean atPosition = false;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(LowerCAN.ELEVATOR_LEAD, LowerCAN.LOWER_CAN);
       
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 424.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 4.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.58;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.16;
        motorConfig.Slot0.kA = 0.004;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 42.75;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", () -> leadMotor.clearStickyFaults());


        leadPosition = leadMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
  
        PhoenixUtil.run("Set Elevator Signal Frequencies", () ->
        BaseStatusSignal.setUpdateFrequencyForAll(
            200,
            leadPosition,
            leadVelocity
        )
    );
        PhoenixUtil.run("Set Elevator Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, leadMotor)
        );

        positionControl = new MotionMagicVoltage(0.0);
       


        Tunable.pidController("elevator/pid", leadMotor);
        Tunable.motionProfile("elevator/motion", leadMotor);

    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(leadPosition, leadVelocity);
    }

    // *************** Helper Functions ***************

    @NotLogged
    public boolean atPosition() {
        return atPosition;
    }

    public boolean safeForIntake() {
        return getPosition() <= ElevatorPosition.INTAKE.radians() + CLOSEST_TOLERANCE.value();
    }

    /**
     * Gets the elevator's current position, in rotations.
     */
    private double getPosition() {
        return (
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity)));
    }

    // *************** Commands ***************

    /**
     * Goes to a scoring position.
     * @param selection The reef selection helper.
     * @param safe If the elevator is safe to move.
     */
    public Command score(ReefSelection selection, BooleanSupplier safe) {
     

        return goTo(
            () -> {
                switch (selection.getLevel()) {
                    case 2:
                        return ElevatorPosition.L2;
                    case 3:
                        return ElevatorPosition.L3;
                    case 4:
                        return ElevatorPosition.L4;
                    default:
                        return ElevatorPosition.DOWN;
                }
            },
            () -> 0.0, 
            safe
        );}

    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    public Command goTo(ElevatorPosition position, BooleanSupplier safe) {
        return goTo(() -> position, () -> 0.0, safe);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    private Command goTo(Supplier<ElevatorPosition> position, DoubleSupplier fudge, BooleanSupplier safe) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);

        return commandBuilder("Elevator.goTo()")
            .onInitialize(() -> holdPosition.value = -1.0)
            .onExecute(() -> {
            
                ElevatorPosition targetPos = position.get();
                double target = targetPos.radians();
                double currentPosition = getPosition();
                atPosition = Math.abs(currentPosition - target) < AT_POSITION_TOLERANCE.value();

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        ElevatorPosition close = ElevatorPosition.closest(currentPosition);
                        holdPosition.value = close != null ? close.radians() : currentPosition;
                    }

                    target = holdPosition.value;
                } else {
                    holdPosition.value = -1.0;
                }

                if (currentPosition - ZERO_TOLERANCE.value() <= 0.0 && target - ZERO_TOLERANCE.value() <= 0.0) {
                    leadMotor.stopMotor();
                } else {
                    leadMotor.setControl(positionControl.withPosition(target + fudge.getAsDouble()));
                }
            })
            .onEnd(() -> {
                leadMotor.stopMotor();
                atPosition = false;
            });
    }

    public Command thing() {
        return commandBuilder().onInitialize(() -> {}).onExecute(() -> {}).isFinished(false).onEnd(() -> {});
    }
}
