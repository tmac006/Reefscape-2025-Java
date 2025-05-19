package org.team1507.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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
    private final AtomicBoolean hasAlg = new AtomicBoolean(false);

    public AlgMech() {
        // Create new motors and digital inputs
        clawMotor = new SparkMax(Constants.RioCAN.ALG_MOTOR, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //Warm Up enum
    ALGSpeed.INTAKE.speeds();
}


@Override
public void periodic() {
    SmartDashboard.putBoolean("ALG Photoeye", !photoeye.get());
}

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