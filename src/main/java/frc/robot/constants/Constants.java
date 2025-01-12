package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public static class SwerveConstants {

        public static final double[] TRANSLATION_PP_PID = {10, 0, 0};
        public static final double[] ROTATION_PP_PID = {7, 0, 0};

    }

    public static class ElevatorConstants {

        public static final int ELEVATOR_LEFT = 2;
        public static final int ELEVATOR_RIGHT = 3;

        public static final boolean leftInverted = false;

        public static final double CORAL_STATION_ENCODER_TICKS = 0;
        public static final double L1_ENCODER_TICKS = 100;
        public static final double L2_ENCODER_TICKS = 200;
        public static final double L3_ENCODER_TICKS = 300;
        public static final double MAX_ENCODER_TICKS = 400; // TODO FIGURE THESE OUT

        public static final double[] ELEVATOR_PID = {1, 0, 0};

    }

    public static class CameraConstants {

        public static class AprilTags {

            // facing the field elements (or from the driver station for the stages)
            public static final int RIGHT_BLUE_SOURCE = 1; 
            public static final int LEFT_BLUE_SOURCE = 2;
            public static final int RIGHT_RED_SPEAKER = 3;
            public static final int MIDDLE_RED_SPEAKER = 4; // target
            public static final int RED_AMP = 5; // target
            public static final int BLUE_AMP = 6; // target
            public static final int MIDDLE_BLUE_SPEAKER = 7; // target
            public static final int LEFT_BLUE_SPEAKER = 8;
            public static final int RIGHT_RED_SOURCE = 9;
            public static final int LEFT_RED_SOURCE = 10;
            public static final int STAGE_LEFT_RED_TRAP = 11;
            public static final int STAGE_RIGHT_RED_TRAP = 12;
            public static final int CENTER_STAGE_RED_TRAP = 13;
            public static final int CENTER_STAGE_BLUE_TRAP = 14;
            public static final int STAGE_LEFT_BLUE_TRAP = 15;
            public static final int STAGE_RIGHT_BLUE_TRAP = 16;
        
            public AprilTags() {
            }
        }    

        public static final String CAMERA_1_NAME = "testCam";

        public static final Transform3d CAMERA_1_POS = new Transform3d(
            new Translation3d(0, 0, 0), // forward from center, up from center, right from center
            new Rotation3d(0, 0, 0) // 0 0 0 is facing forward, positive rotates that axis clockwise
        );

    }
    
}
