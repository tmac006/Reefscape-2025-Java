package org.team1507.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import org.team1507.lib.util.Tunable;
import org.team1507.lib.util.Tunable.TunableDouble;
import org.team1507.lib.util.command.GRRSubsystem;
import org.team1507.robot.Constants.RioCAN;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

@Logged
public final class Claw extends GRRSubsystem {

    private final DigitalInput beambreak = new DigitalInput(RioCAN.CLAW_BEAM_BREAK);
    private final SparkMax clawMotor;


    private static enum ClawSpeed {
        INTAKE(-1),
        OUTTAKE(1),
        CREEP_BACKWARDS(-0.2);       
    

    private TunableDouble speeds;

    private ClawSpeed(double speeds) {
        this.speeds = Tunable.doubleValue("ClawSpeed/speeds/" + name(), speeds);
    }

    private double speeds() {
        return speeds.value();
    }
 }

  private final AtomicBoolean hasCoral = new AtomicBoolean(false);

 public Claw() {
    // Create new motors and digital inputs
    clawMotor = new SparkMax(1, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config
     .inverted(true)
     .idleMode(IdleMode.kBrake)
     .smartCurrentLimit(40);

     clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
     //Warm Up enum
  ClawSpeed.INTAKE.speeds();
 }
  
 @Override
public void periodic()
{
  SmartDashboard.putBoolean("beam break" , beamBroken());
}


 @NotLogged
 public boolean hasCoral() {
     return hasCoral.get();
 }

 @NotLogged
 public boolean noCoral() {
     return !hasCoral();
 }

 public boolean beamBroken() {
    return !beambreak.get();
}

// *************** Commands ***************

public Command setHasCoral(boolean hasCoral) {
    return runOnce(() -> this.hasCoral.set(hasCoral))
        .ignoringDisable(true)
        .withName("Intake.setHasCoral(" + hasCoral + ")");
}

public enum ClawState {
    RunClawFull,
    Sensor1,
    RunClawCreep,
    NotDetected,
    RunBackwardUntilClawSeen,
    StopMotor,
    EndState
}
    
private ClawState currentState = ClawState.RunClawFull;
private final Timer delay = new Timer();


public Command asCommand(double intakePower) {
    // Initialize state on schedule start
    InstantCommand init = new InstantCommand(() -> {
        currentState = ClawState.RunClawFull;
        setHasCoral(false);
        delay.reset();
    });

    // Use Command type instead of RunCommand to accommodate decorated commands
    Command runner = new RunCommand(() -> {
        switch (currentState) {
            case RunClawFull:
                clawMotor.set(intakePower);
                currentState = ClawState.Sensor1;
                break;
            case Sensor1:
                if (beambreak.get()) { // Use actual sensor reference
                    currentState = ClawState.RunClawCreep;
                }
                break;
            case RunClawCreep:
                clawMotor.set(ClawSpeed.CREEP_BACKWARDS.speeds());
                delay.reset();
                currentState = ClawState.NotDetected;
                break;
            case NotDetected:
                if (!beambreak.get()) { // Check sensor again
                    currentState = ClawState.RunBackwardUntilClawSeen;
                }
                break;
            case RunBackwardUntilClawSeen:
                clawMotor.set(ClawSpeed.OUTTAKE.speeds());
                if (beambreak.get()) {
                    currentState = ClawState.StopMotor;
                }
                break;
            case StopMotor:
                clawMotor.stopMotor();
                setHasCoral(true);
                currentState = ClawState.EndState;
                break;
            case EndState:
                break;
        }
    }, this) // Pass subsystem requirement
    .until(() -> currentState == ClawState.EndState); // Now returns a generic Command

    return init.andThen(runner)
               .finallyDo(interrupted -> clawMotor.stopMotor());
}

//Outtake commands
public Command barf(double durationSeconds) {
    return Commands.run(
            () -> clawMotor.set(ClawSpeed.OUTTAKE.speeds()), // Use enum 
            this
        )
        .withTimeout(durationSeconds)
        .finallyDo(interrupted -> {
            clawMotor.stopMotor();
            hasCoral.set(false);
        })
        .withName("BarfFor" + durationSeconds + "Seconds");
}



public Command thing() {
    return commandBuilder().onInitialize(() -> {}).onExecute(() -> {}).isFinished(false).onEnd(() -> {});
}

}

