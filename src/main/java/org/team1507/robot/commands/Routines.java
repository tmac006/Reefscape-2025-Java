// package org.team1507.robot.commands;

// import static edu.wpi.first.wpilibj2.command.Commands.*;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.epilogue.Logged.Strategy;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import java.util.function.BooleanSupplier;
// import org.team1507.lib.util.Tunable;
// import org.team1507.lib.util.Tunable.TunableBoolean;
// import org.team1507.robot.Robot;
// import org.team1507.robot.subsystems.Swerve;
// import org.team1507.robot.util.ReefSelection;

// /**
//  * The Routines class contains command compositions, such as sequences
//  * or parallel command groups, that require multiple subsystems.
//  */
// @SuppressWarnings("unused")
// @Logged(strategy = Strategy.OPT_IN)
// public final class Routines {

//     private static final TunableBoolean AUTO_DRIVE = Tunable.booleanValue("routines/autoDrive", true);

//     private final Robot robot;

//     private final Swerve swerve;

//     private final ReefSelection selection;

//     public Routines(Robot robot) {
//         this.robot = robot;
//         swerve = robot.swerve;
//         selection = robot.selection;
//     }

//     /**
//      * Scores a coral, with driver assists. Intended to be used during teleop control.
//      * @param runManual A boolean supplier that when {@code true} will force the
//      *                  goose beak to spit, even if a pipe is not detected.
//      * @param allowGoosing If the goose neck is allowed to goose around.
//      */
//     public Command assistedScore(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
//         return either(
//             swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft),
//             sequence(
//                 swerve.apfDrive(selection::isLeft, robot::readyToScore, selection::isL4).until(gooseNeck::noCoral),
//                 swerve.drive(robot::driverX, robot::driverY, robot::driverAngular)
//             ),
//             () -> !AUTO_DRIVE.value() || gooseNeck.noCoral()
//         ).withName("Routines.assistedScore()");
//     }
    

// }
