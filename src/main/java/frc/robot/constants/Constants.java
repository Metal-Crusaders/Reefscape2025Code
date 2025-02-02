package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public static class OIConstants {
        public static int XBOX_A = 1;
        public static int XBOX_B = 2;
        public static int XBOX_X = 3;
        public static int XBOX_Y = 4;
        public static int XBOX_LB = 5;
        public static int XBOX_RB = 6;
        public static int XBOX_BACK = 7;
        public static int XBOX_START = 8;
        public static int XBOX_LEFT_STICK = 9;
        public static int XBOX_RIGHT_STICK = 10;
    }

    public static class SwerveConstants {

        public static final double DRIVE_KP = 0.17105;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KS = 0.045286;
        public static final double DRIVE_KV = 0.1141075;
        public static final double DRIVE_KA = 0.005900075;
        
        public static final double TURNING_KP = 75.0;
        public static final double TURNING_KI = 0.0;
        public static final double TURNING_KD = 0.0;
        public static final double TURNING_KS = 0.0;
        public static final double TURNING_KV = 0.0;
        public static final double TURNING_KA = 0.0;

        public static final double ROTATION_KP = 0.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
        public static final double ROTATION_KS = 0.0;
        public static final double ROTATION_KV = 0.0;
        public static final double ROTATION_KA = 0.0;

        public static final double TRANSLATION_PP_KP = 4.0;
        public static final double TRANSLATION_PP_KI = 0.0;
        public static final double TRANSLATION_PP_KD = 0.0;

        public static final double ROTATION_PP_KP = 2.5;
        public static final double ROTATION_PP_KI = 0.0;
        public static final double ROTATION_PP_KD = 0.0;

        public static final double CLOSE_TRANSLATION_PP_KP = 3.0;
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 2.0;
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;

        public static final double MAX_LINEAR_VELOCITY = 2.0;
        public static final double MAX_LINEAR_ACCELERATION = 1.5;
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;

    }

    public static class AutoDriveConstants {

        public static final Pose2d[] BLUE_REEF_POSES = {
            new Pose2d(2.823, 4.000, new Rotation2d(0 * Math.PI / 180.0)),
            new Pose2d(3.719, 2.614, new Rotation2d(60 * Math.PI / 180.0)),
            new Pose2d(5.430, 2.640, new Rotation2d(120 * Math.PI / 180.0)),
            new Pose2d(6.131, 4.000, new Rotation2d(180 * Math.PI / 180.0)),
            new Pose2d(5.384, 5.406, new Rotation2d(-120 * Math.PI / 180.0)),
            new Pose2d(3.689, 5.515, new Rotation2d(-60 * Math.PI / 180.0))
        };

        public static final Pose2d[] RED_REEF_POSES = {
            new Pose2d(2.823 + 8.553921, 4.000, new Rotation2d(0 * Math.PI / 180.0)),
            new Pose2d(3.719 + 8.553921, 2.614, new Rotation2d(60 * Math.PI / 180.0)),
            new Pose2d(5.430 + 8.553921, 2.640, new Rotation2d(120 * Math.PI / 180.0)),
            new Pose2d(6.000 + 8.553921, 4.000, new Rotation2d(180 * Math.PI / 180.0)),
            new Pose2d(5.384 + 8.553921, 5.406, new Rotation2d(-120 * Math.PI / 180.0)),
            new Pose2d(3.689 + 8.553921, 5.515, new Rotation2d(-60 * Math.PI / 180.0))
        };

        public static final double[][] ADDITIONS = {
            {0.342, 0}, // LEFT ADDITION
            {0.342, -0.348}  // RIGHT ADDITION
        };

    }

    public static class ElevatorConstants {

        public static final int ELEVATOR_LEFT = 2;
        public static final int ELEVATOR_RIGHT = 3;

        public static final boolean LEFT_INVERTED = false;

        public static final double CORAL_STATION_ENCODER_TICKS = 0;
        public static final double L1_ENCODER_TICKS = 100;
        public static final double L2_ENCODER_TICKS = 200;
        public static final double L3_ENCODER_TICKS = 300;
        public static final double MAX_ENCODER_TICKS = 400; // TODO FIGURE THESE OUT

        public static final double[] ELEVATOR_PID = {0.15, 0, 0, 0.5}; // kP, kI, kD, kF
        // TODO FIGURE THE PID OUT TOO, using Cranberry Alarm's initial values for now

        public static final double MAX_VELOCITY = 10.0;
        public static final double MAX_ACCELERATION = 10.0;
        public static final double RAMP_RATE = 20.0;

        public static final double ELEVATOR_TICKS_DEADBAND = 10.0;

    }

    public static class CoralShooterConstants {

        public static final int CORAL_LEFT = 17;
        public static final int CORAL_RIGHT = 16;

        public static final boolean LEFT_INVERTED = false;

        public static final double RAMP_RATE = 20.0;

        public static final int CORAL_BEAM_ID = 1;
        public static final int CORAL_BEAM_SENSOR_ID = 2;

    }

    public static class AlgaeClawConstants {

        public static final int ALGAE_CLAW_ID = 4;
        public static final int ALGAE_PIVOT_ID = 5;

        public static final boolean CLAW_INVERTED = false; 
        public static final boolean PIVOT_INVERTED = false; 

        public static final double[] PIVOT_PID = {0.5, 0, 0, 0.5}; // kP, kI, kD, kF
        // TODO preset PID stuff!


        public static final double RAMP_RATE = 20.0;

        // TODO figure this out too
        public static final double PIVOT_IN_TICKS = 0.0;
        public static final double PIVOT_OUT_TICKS = 0.0;
        public static final double PIVOT_TICKS_DEADBAND = 10.0;

        public static final double CURRENT_MAX = 30.0; // TEST

    }

    public static class CameraConstants {

        public static final String CAMERA_1_NAME = "testCam";

        public static final Transform3d CAMERA_1_POS = new Transform3d(
            new Translation3d(0.36, -0.051, 0), // forward from center, up from center, right from center
            new Rotation3d(0, 10.0 * Math.PI / 180.0, 0) // 0 0 0 is facing forward, positive rotates that axis clockwise
        );

    }
    
}
