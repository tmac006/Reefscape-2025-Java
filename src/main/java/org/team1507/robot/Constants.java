package org.team1507.robot;

import org.team1507.robot.util.Vision;
import org.team1507.robot.util.Vision.CameraConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double VOLTAGE = 12.0;

    public static final int DRIVER = 0;
    public static final int CO_DRIVER = 1;


    //Put cameras back on, put limelight on running photon vision if we want 3 (would be cool)
    public static final CameraConfig[] CAMERAS = {
        new CameraConfig(
            "middle",
            new Translation3d(0.354, 0.0, 0.215),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(0.0))
        ),
        new CameraConfig(
            "left",
            new Translation3d(0.316, 0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(45.0))
        ),
        new CameraConfig(
            "right",
            new Translation3d(0.316, -0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(-45.0))
        )
    };

    public static final class LowerCAN {

        public static final String LOWER_CAN = "LowerCAN";

        // Swerve
        public static final int FL_MOVE = 2;
        public static final int FL_TURN = 3;
        public static final int FR_MOVE = 4;
        public static final int FR_TURN = 5;
        public static final int BL_MOVE = 6;
        public static final int BL_TURN = 7;
        public static final int BR_MOVE = 8;
        public static final int BR_TURN = 9;

        public static final int FL_ENCODER = 10;
        public static final int FR_ENCODER = 11;
        public static final int BL_ENCODER = 12;
        public static final int BR_ENCODER = 13;

        // Elevator
        public static final int ELEVATOR_LEAD = 20;
    }

    public static final class RioCAN {

 
        public static final int CLAW_MOTOR = 31;
        public static final int CLAW_BEAM_BREAK = 0;

        // Climber
        public static final int CLIMBER_MOTOR = 50;

        // Alg
        public static final int PIVOT_MOTOR = 30;
        public static final int ALG_MOTOR = 51;
        public static final int ALG_PHOTOEYE = 1;
        public static final int ALG_ENCODER = 52;
    }
}
