package org.team1507.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import org.team1507.lib.util.Tunable;
import org.team1507.lib.util.Tunable.TunableDouble;
import org.team1507.lib.util.command.GRRSubsystem;
import org.team1507.lib.util.vendors.PhoenixUtil;
import org.team1507.robot.Constants;
import org.team1507.robot.Constants.RioCAN;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1507.robot.Constants.RioCAN;


public final class AlgMech extends GRRSubsystem {
    
   private final DigitalInput photoeye = new DigitalInput(RioCAN.ALG_PHOTOEYE);
   private final SparkMax clawMotor;    


    private static enum ALGSpeed {
         INTAKE(-1),
         OUTTAKE(1),
         HOLDING(-0.2);
    
    private TunableDouble speeds;

    private ALGSpeed(double speeds) {
        this.speeds = Tunable.doubleValue("ALGSpeed/speeds/" + name(), speeds);
    }

    private double speeds() {
        return speeds.value();
    }
  }

  public static enum PivotPosition {
        HOME(0.0),
        INTAKE(0.0),
        STOW(0.0),
        L1SCORE(0.0),
        L1FLOOR(0.0),
        BARGE(0.0);
    
    
        private final TunableDouble radians;

        private PivotPosition(double radians) {
            this(radians, false);
        }

        private PivotPosition(double radians, boolean scoring) {
            this.radians = Tunable.doubleValue("alg/positions/" + name(), radians);
        }

        public double radians() {
            return radians.value();
        }
        private static PivotPosition closest(double position) {
            PivotPosition closest = null;
            double min = Double.MAX_VALUE;
            for (PivotPosition option : values()) {
                double distance = Math.abs(option.radians() - position);
                if (distance < CLOSEST_TOLERANCE.value() && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
        
  }

  private static final TunableDouble CLOSEST_TOLERANCE = Tunable.doubleValue("alg/closestTolerance", 0.15);
  private static final TunableDouble AT_POSITION_TOLERANCE = Tunable.doubleValue("alg/atPositionTolerance", 0.1);
  private static final TunableDouble ZERO_TOLERANCE = Tunable.doubleValue("alg/zeroTolerance", 0.15);
  private final TalonFX leadMotor;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;

    private final MotionMagicVoltage positionControl;

    private boolean atPosition = false;

    private final AtomicBoolean hasAlg = new AtomicBoolean(false);

    public AlgMech() {
        // Create new spark motor 
        clawMotor = new SparkMax(Constants.RioCAN.ALG_MOTOR, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

        //Create new TalonFX
        leadMotor = new TalonFX(RioCAN.PIVOT_MOTOR);

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

        PhoenixUtil.run("Clear Pivot Sticky Faults", () -> leadMotor.clearStickyFaults());


        leadPosition = leadMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
  
        PhoenixUtil.run("Set Pivot Signal Frequencies", () ->
        BaseStatusSignal.setUpdateFrequencyForAll(
            200,
            leadPosition,
            leadVelocity
        )
    );
        PhoenixUtil.run("Set Pivot Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Pivot CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, leadMotor)
        );

        positionControl = new MotionMagicVoltage(0.0);
       


        Tunable.pidController("pivot/pid", leadMotor);
        Tunable.motionProfile("pivot/motion", leadMotor);


    //Warm Up enum
    ALGSpeed.INTAKE.speeds();
    PivotPosition.STOW.radians();

}


@Override
public void periodic() {
    SmartDashboard.putBoolean("ALG Photoeye", !photoeye.get());
    BaseStatusSignal.refreshAll(leadPosition, leadVelocity);
}

// *************** Pivot Functions ***************

@NotLogged
public boolean atPosition() {
    return atPosition;
}

private double getPosition() {
    return (
        (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity)));
}

// *************** Alg Functions ***************
@NotLogged
public boolean hasAlg() {
    return !hasAlg.get();
}
@NotLogged
public boolean noAlg() {
    return !hasAlg();
}

public boolean photoeyeMade() {
    return !photoeye.get();
}

// Commands

public Command SetHasAlg(boolean hasAlg) {
    return runOnce(() -> this.hasAlg.set(hasAlg))
    .ignoringDisable(true)
    .withName("Intake.setHasAlg(" + hasAlg + ")");

}
}