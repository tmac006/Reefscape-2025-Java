package org.team1507.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1507.lib.util.DisableWatchdog;
import org.team1507.lib.util.Tunable;
// import org.team1507.robot.commands.Autos;
// import org.team1507.robot.commands.Routines;
import org.team1507.robot.subsystems.Swerve;
import org.team1507.robot.util.ReefSelection;

@Logged
public final class Robot extends TimedRobot {

    public final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Swerve swerve;

    public final ReefSelection selection;

    // public final Routines routines;
    // public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DisableWatchdog.in(scheduler, "m_watchdog");
        DisableWatchdog.in(this, "m_watchdog");

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        swerve = new Swerve();

        // Initialize helpers
        selection = new ReefSelection();

        // Initialize compositions
        // routines = new Routines(this);
        // autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Create triggers
        // Trigger gooseAround = driver.x().negate().and(coDriver.a().negate());
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Driver bindings


        driver.leftStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));
      
        
        driver.povLeft().onTrue(swerve.tareRotation());

        // driver.leftBumper().onTrue(selection.setLeft()).whileTrue(routines.assistedScore(driver.y(), gooseAround));
        // driver.rightBumper().onTrue(selection.setRight()).whileTrue(routines.assistedScore(driver.y(), gooseAround));

      

        // Set thread priority
        waitSeconds(5.0)
            .until(DriverStation::isEnabled)
            .andThen(() -> {
                Threads.setCurrentThreadPriority(true, 10);
                SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
            })
            .schedule();
    }

    /**
     * Returns the current match time in seconds.
     */
    public double matchTime() {
        return Math.max(DriverStation.getMatchTime(), 0.0);
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
        Epilogue.update(this);
       
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
