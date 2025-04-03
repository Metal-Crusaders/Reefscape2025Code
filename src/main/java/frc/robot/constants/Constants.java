package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

        public static int APAC1 = 4;
        public static int APAC2 = 5;

        public static double SCORE_DEBOUNCING_TIME = 0.05;
        public static double REST_DEBOUNCING_TIME = 0.05;
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

        public static final double TRANSLATION_PP_KP = 4.5;
        public static final double TRANSLATION_PP_KI = 0.0;
        public static final double TRANSLATION_PP_KD = 0.0;

        public static final double ROTATION_PP_KP = 2.5;
        public static final double ROTATION_PP_KI = 0.0;
        public static final double ROTATION_PP_KD = 0.0;

        public static final double CLOSE_TRANSLATION_PP_KP = 1.5;
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 2.0;
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;

        public static final double REEF_ROTATION_PID_KP = 2.0;
        public static final double REEF_ROTATION_PID_KI = 0.0;
        public static final double REEF_ROTATION_PID_KD = 0.0;

        public static final double MAX_LINEAR_VELOCITY = 2.5;
        public static final double MAX_LINEAR_ACCELERATION = 2.0;
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;

    }

    public static class AutoDriveConstants {

        // RED_REEF_POSES array for ID 6 through 11
        public static final Pose2d[] RED_REEF_POSES = new Pose2d[] {
            new Pose2d(13.917, 2.859, new Rotation2d(2.0943951023931957)), // ID 6
            new Pose2d(14.49476434251882, 4.182964006546419, new Rotation2d(3.141592653589793)), // ID 7
            new Pose2d(13.636158342518822, 5.344784006546417, new Rotation2d(-2.0943951023931957)), // ID 8
            new Pose2d(12.200804, 5.182639999999999, new Rotation2d(-1.0471975511965979)), // ID 9
            new Pose2d(11.623039657481177, 3.8586759934535824, new Rotation2d(0.0)), // ID 10
            new Pose2d(12.481645657481177, 2.696855993453582, new Rotation2d(1.0471975511965974)) // ID 11
        };

        public static final Pose2d[][] RED_REEF_NEIGHBORS = new Pose2d[][] {
            {RED_REEF_POSES[5], RED_REEF_POSES[1]}, // ID 6 -> Left: ID 11, Right: ID 7
            {RED_REEF_POSES[0], RED_REEF_POSES[2]}, // ID 7 -> Left: ID 6, Right: ID 8
            {RED_REEF_POSES[1], RED_REEF_POSES[3]}, // ID 8 -> Left: ID 7, Right: ID 9
            {RED_REEF_POSES[2], RED_REEF_POSES[4]}, // ID 9 -> Left: ID 8, Right: ID 10
            {RED_REEF_POSES[3], RED_REEF_POSES[5]}, // ID 10 -> Left: ID 9, Right: ID 11
            {RED_REEF_POSES[4], RED_REEF_POSES[0]}  // ID 11 -> Left: ID 10, Right: ID 6
        };

        public static final int[] RED_REEF_IDS = {6, 7, 8, 9, 10, 11};

        // BLUE_REEF_POSES array for ID 17 through 22
        public static final Pose2d[] BLUE_REEF_POSES = new Pose2d[] {
            new Pose2d(3.6313519999999997, 5.182639999999999, new Rotation2d(-1.0471975511965979)), // ID 19
            new Pose2d(3.0533336574811782, 3.8586759934535824, new Rotation2d(0.0)), // ID 18
            new Pose2d(3.9121936574811764, 2.696855993453582, new Rotation2d(1.0471975511965974)), // ID 17
            new Pose2d(5.347293999999999, 2.859, new Rotation2d(2.0943951023931957)), // ID 22
            new Pose2d(5.925312342518822, 4.182964006546419, new Rotation2d(3.141592653589793)), // ID 21
            new Pose2d(5.066452342518822, 5.344784006546417, new Rotation2d(-2.0943951023931957)) // ID 20
        };

        public static final Pose2d[][] BLUE_REEF_NEIGHBORS = new Pose2d[][] {
            {BLUE_REEF_POSES[5], BLUE_REEF_POSES[1]}, // ID 19 -> Left: ID 20, Right: ID 18
            {BLUE_REEF_POSES[0], BLUE_REEF_POSES[2]}, // ID 18 -> Left: ID 19, Right: ID 17
            {BLUE_REEF_POSES[1], BLUE_REEF_POSES[3]}, // ID 17 -> Left: ID 18, Right: ID 22
            {BLUE_REEF_POSES[2], BLUE_REEF_POSES[4]}, // ID 22 -> Left: ID 17, Right: ID 21
            {BLUE_REEF_POSES[3], BLUE_REEF_POSES[5]}, // ID 21 -> Left: ID 22, Right: ID 20
            {BLUE_REEF_POSES[4], BLUE_REEF_POSES[0]}  // ID 20 -> Left: ID 21, Right: ID 19
        };

        public static final int[] BLUE_REEF_IDS = {19, 18, 17, 22, 21, 20};

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4.0);
        public static final Translation2d RED_REEF_CENTER = new Translation2d(13, 4.0);

        public static final double RED_OFFSET = 8.58;

        public static final Pose2d[] BLUE_CORAL_STATION_POSES = {
            new Pose2d(1.300, 1.000, new Rotation2d(55 * Math.PI / 180.0)),
            new Pose2d(1.300, 7.000, new Rotation2d(-55 * Math.PI / 180.0))
        };

        public static final Pose2d[] RED_CORAL_STATION_POSES = {
            new Pose2d(16.300, 1.000, new Rotation2d(125 * Math.PI / 180.0)),
            new Pose2d(16.300, 7.000, new Rotation2d(-125 * Math.PI / 180.0))
        };

        public static final double[][] ADDITIONS = {
            {0.4, 0.08}, // LEFT ADDITION
            {0.42, -0.3}  // RIGHT ADDITION
        };

        public static final double[] POSE_ADDITION = {
            -0.4, 0.0
        };

        public static final double[] ALGAE_ADDITION = {
            -0.5, 0.0
        };

        public static final int[] GOOD_BLUE_APRIL_TAGS = {
            17, 18, 19, 20, 21, 22
        };

        public static final int[] GOOD_RED_APRIL_TAGS = {
            6, 7, 8, 9, 10, 11
        };

        public static final Translation2d[] BORDER_POSES = {
            new Translation2d(0.000, 1.225),
            new Translation2d(0.000, 6.800),
            new Translation2d(1.750, 8.000),
            new Translation2d(16.000, 8.000),
            new Translation2d(17.548, 6.800),
            new Translation2d(17.548, 1.225),
            new Translation2d(16.000, 0.000),
            new Translation2d(1.750, 0.000)
        };

    }

    public static class ElevatorConstants {

        public static final int ELEVATOR_LEFT = 17;
        public static final int ELEVATOR_RIGHT = 18;

        public static final boolean LEFT_INVERTED = true;

        public static final double L1_ENCODER_TICKS = 0;
        public static final double PROCESSOR_ALGAE_TICKS = 10;
        public static final double L2_ENCODER_TICKS = 14.8095;
        public static final double L3_ENCODER_TICKS = 43.8095;
        public static final double ALGAE_SPOKE_OFFSET = 27;
        public static final double LOW_ALGAE_ENCODER_TICKS = 13.8095 + ALGAE_SPOKE_OFFSET;
        public static final double HIGH_ALGAE_ENCODER_TICKS = 41.8095 + ALGAE_SPOKE_OFFSET;
        public static final double MAX_ENCODER_TICKS = 69;

        public static final double[] ELEVATOR_PID = {0.1, 0, 0, 0.0}; // kP, kI, kD, kF
        // TODO FIGURE THE PID OUT TOO, using Cranberry Alarm's initial values for now

        public static final double MAX_PERCENT_SPEED = 0.85;
        public static final double MAX_VELOCITY = 0.00005;
        public static final double MAX_ACCELERATION = 2.0;
        public static final double RAMP_RATE = 20.0;

        public static final double ELEVATOR_TICKS_DEADBAND = 5.0;

    }

    public static class CoralShooterConstants {

        public static final int CORAL_LEFT = 13;
        public static final int CORAL_RIGHT = 14;

        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final double RAMP_RATE = 20.0;

        public static final int CORAL_BEAM_ID = 4;
        public static final int CORAL_BEAM_SENSOR_ID = 3;

    }

    public static class AlgaeClawConstants {

        public static final int ALGAE_CLAW_ID = 15;
        public static final int ALGAE_PIVOT_ID = 16;

        public static final boolean CLAW_INVERTED = false; 
        public static final boolean PIVOT_INVERTED = false; 

        public static final double[] PIVOT_PID = {0.7 / 950.0, 0.0, 0, 0.0}; // kP, kI, kD, kF
        // TODO preset PID stuff!

        // TODO figure this out too
        public static final double PIVOT_IN_TICKS = 100.0;
        public static final double PIVOT_OUT_TICKS = 950.97725;
        public static final double PIVOT_TICKS_DEADBAND = 50.0;

        public static final double CURRENT_MAX = 30.0; // TEST

        public static final double MAX_VELOCITY = 210;
        public static final double MAX_ACCELERATION = 105;
        public static final double RAMP_RATE = 20.0;

        public static final double PROXIMITY_THRESHOLD = 150.0; // TODO tweak this
        // public static final double BLUE_THRESHOLD = 200.0; // TODO tweak this

    }

    public static class CameraConstants {

        // CMAERA TO THE FARTHEST RIGHT, photonvision1.local at 10.52.93.11
        public static final String CAMERA_1_NAME = "testCam2";

        public static final Transform3d CAMERA_1_POS = new Transform3d(
            new Translation3d(0.2250948, 0.166770812, -0.2268097534), // forward from center, up from center, right from center
            new Rotation3d(0, 15.0 * Math.PI / 180.0, 45.0 * Math.PI / 180.0) // 0 0 0 is facing forward, positive rotates that axis clockwise
        );

        // CAMERA TO THE CENTER, photonvision2.local at 10.52.93.12
        public static final String CAMERA_2_NAME = "testCam";

        public static final Transform3d CAMERA_2_POS = new Transform3d(
            new Translation3d(0.3282442, 0.1129792, -0.0635), // forward from center, up from center, right from center
            new Rotation3d(0, 20.0 * Math.PI / 180.0, 0) // 0 0 0 is facing forward, positive rotates that axis clockwise
        );

        // CAMERA TO THE FARTHEST LEFT, photonvision2.local at 10.52.93.12 (FIGURE THIS OUT)
        public static final String CAMERA_3_NAME = "testCam3";

        public static final Transform3d CAMERA_3_POS = new Transform3d(
            new Translation3d(0.2837942, 0.1712214, 0.2446655), // forward from center, up from center, right from center
            new Rotation3d(0, 15.0 * Math.PI / 180.0, -45.266950 * Math.PI / 180.0) // 0 0 0 is facing forward, positive rotates that axis clockwise
        );

    }

    public static class FieldConstants {

        public static final Pose2d[] FIELD_BORDERS = {
            new Pose2d(0, 0, new Rotation2d(0)),
        };

        public static final Pose2d[] BLUE_REEF_BORDERS = {
            new Pose2d(0, 0, new Rotation2d(0)),
        };

        public static final Pose2d[] RED_REEF_BORDERS = {
            new Pose2d(0, 0, new Rotation2d(0)),
        };

    }
    
}
