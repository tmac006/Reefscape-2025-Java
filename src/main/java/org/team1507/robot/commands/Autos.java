// package org.team1507.robot.commands;

// import static edu.wpi.first.wpilibj2.command.Commands.*;
// import static org.team1507.robot.util.Field.FLIPPER;
// import static org.team1507.robot.util.Field.ReefLocation.*;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.epilogue.Logged.Strategy;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import java.util.function.Function;
// import java.util.function.Supplier;
// import org.team1507.lib.util.Alliance;
// import org.team1507.lib.util.AutoChooser;
// import org.team1507.lib.util.Tunable;
// import org.team1507.lib.util.Tunable.TunableDouble;
// import org.team1507.robot.Robot;
// import org.team1507.robot.subsystems.Swerve;
// import org.team1507.robot.util.Field;
// import org.team1507.robot.util.Field.ReefLocation;
// import org.team1507.robot.util.ReefSelection;

// /**
//  * The Autos class declares autonomous modes, and adds them
//  * to the dashboard to be selected by the drive team.
//  */
// @SuppressWarnings("unused")
// @Logged(strategy = Strategy.OPT_IN)
// public final class Autos {

//     private static final TunableDouble INTAKE_SLOWDOWN = Tunable.doubleValue("autos/intakeSlowdown", 8.7);
//     private static final TunableDouble INTAKE_ROT_DELAY = Tunable.doubleValue("autos/intakeRotDelay", 0.6);
//     private static final TunableDouble AVOID_SLOWDOWN = Tunable.doubleValue("autos/avoidSlowdown", 8.9);
//     private static final TunableDouble AVOID_TOLERANCE = Tunable.doubleValue("autos/avoidTolerance", 0.25);

//     private final Robot robot;

//     private final Elevator elevator;
//     private final GooseNeck gooseNeck;
//     private final Intake intake;
//     private final Lights lights;
//     private final Swerve swerve;

//     private final Routines routines;
//     private final ReefSelection selection;

//     private final AutoChooser chooser;

//     public Autos(Robot robot) {
//         this.robot = robot;

//         elevator = robot.elevator;
//         gooseNeck = robot.gooseNeck;
//         intake = robot.intake;
//         lights = robot.lights;
//         swerve = robot.swerve;

//         selection = robot.selection;
//         routines = robot.routines;

//         // Create the auto factory
//         chooser = new AutoChooser("Autos");
//         chooser.bind(robot.scheduler);

//         // Add autonomous modes to the dashboard
//         chooser.add("For Piece Left", forPiece(true));
//         chooser.add("For Piece Right", forPiece(false));
//         chooser.add("Sneaky Two Left", sneakyTwo(true));
//         chooser.add("Sneaky Two Right", sneakyTwo(false));
//         chooser.add("Stinky One Left", stinkyOne(true));
//         chooser.add("Stinky One Right", stinkyOne(false));
//         chooser.add("Test 2", test2());
//     }

//     private Command forPiece(boolean left) {
//         return parallel(
//             sequence(
//                 pickupCycle(left ? I : F, left),
//                 pickupCycle(left ? J : E, left),
//                 pickupCycle(left ? K : D, left),
//                 pickupCycle(left ? L : C, left),
//                 pickupCycle(left ? A : B, left)
//             ),
//             sequence(
//                 routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
//                 routines.intake()
//             ).repeatedly()
//         ).beforeStarting(parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true)));
//     }

//     private Command sneakyTwo(boolean left) {
//         return parallel(
//             sequence(
//                 score(left ? G : H, left),
//                 avoid(left),
//                 pickup(G, left),
//                 avoid(left),
//                 score(left ? H : G, left),
//                 avoid(left),
//                 swerve.stop(false)
//             ),
//             sequence(
//                 routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
//                 routines.intake()
//             ).repeatedly()
//         ).beforeStarting(parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true)));
//     }

//     private Command stinkyOne(boolean left) {
//         return sequence(
//             deadline(waitSeconds(3.0), swerve.stop(false), routines.stow()),
//             parallel(
//                 sequence(score(left ? G : H, left), avoid(left), swerve.stop(false)),
//                 sequence(
//                     routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
//                     routines.stow()
//                 )
//             )
//         ).beforeStarting(parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true)));
//     }

//     private Command test2() {
//         Function<ReefLocation, Command> goReef = reefLocation ->
//             swerve
//                 .apfDrive(reefLocation, robot::readyToScore, selection::isL4)
//                 .withDeadline(sequence(waitUntil(() -> !swerve.wildlifeConservationProgram()), waitSeconds(0.75)));

//         Function<Boolean, Command> goIntake = left ->
//             swerve.apfDrive(
//                 FLIPPER.flipped(Field.STATION_BACKWARDS, left),
//                 INTAKE_SLOWDOWN::value,
//                 AVOID_TOLERANCE::value
//             );

//         return sequence(
//             swerve.resetPose(FLIPPER.flipped(new Pose2d(4.0, 2.0, Rotation2d.kZero))),
//             goReef.apply(ReefLocation.A),
//             avoid(true),
//             goReef.apply(ReefLocation.E),
//             goIntake.apply(true),
//             goReef.apply(ReefLocation.B),
//             avoid(false),
//             goReef.apply(ReefLocation.K),
//             goIntake.apply(false),
//             goReef.apply(ReefLocation.B),
//             avoid(true)
//         );
//     }

//     private Command pickupCycle(ReefLocation reefLocation, boolean left) {
//         return sequence(score(reefLocation, left), pickup(reefLocation, left));
//     }

//     private Command score(ReefLocation reefLocation, boolean left) {
//         return deadline(
//             sequence(
//                 waitUntil(gooseNeck::hasCoral),
//                 waitUntil(() -> !swerve.wildlifeConservationProgram() && gooseNeck.noCoral())
//             ),
//             swerve.apfDrive(reefLocation, robot::readyToScore, selection::isL4)
//         );
//     }

//     private Command pickup(ReefLocation start, boolean left) {
//         Supplier<Pose2d> station = FLIPPER.flipped(start.back ? Field.STATION_BACKWARDS : Field.STATION_FORWARDS);
//         Timer timer = new Timer();

//         return swerve
//             .apfDrive(
//                 () -> {
//                     Pose2d pose = station.get();
//                     if (!timer.hasElapsed(INTAKE_ROT_DELAY.value())) {
//                         return new Pose2d(
//                             pose.getX(),
//                             pose.getY(),
//                             Alliance.isBlue() ? start.side : start.side.rotateBy(Rotation2d.kPi)
//                         );
//                     } else {
//                         return pose;
//                     }
//                 },
//                 INTAKE_SLOWDOWN::value
//             )
//             .until(() -> intake.coralDetected() || gooseNeck.hasCoral())
//             .beforeStarting(timer::restart);
//     }

//     private Command avoid(boolean left) {
//         return swerve.apfDrive(
//             FLIPPER.flipped(Field.AVOID_LOCATION, left),
//             AVOID_SLOWDOWN::value,
//             AVOID_TOLERANCE::value
//         );
//     }
// }
