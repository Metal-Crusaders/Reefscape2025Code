package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public static class SwerveConstants {

        public static final double[] TRANSLATION_PP_PID = {10, 0, 0};
        public static final double[] ROTATION_PP_PID = {7, 0, 0};

    }

    public static class AutoDriveConstants {

        public static final Pose2d[] BLUE_REEF_POSES = {
            new Pose2d(2.823, 4.000, new Rotation2d(0 * Math.PI / 180.0)),
            new Pose2d(3.719, 2.614, new Rotation2d(60 * Math.PI / 180.0)),
            new Pose2d(5.430, 2.640, new Rotation2d(120 * Math.PI / 180.0)),
            new Pose2d(6.000, 4.000, new Rotation2d(180 * Math.PI / 180.0)),
            new Pose2d(5.432, 5.132, new Rotation2d(-120 * Math.PI / 180.0)),
            new Pose2d(3.689, 5.515, new Rotation2d(-60 * Math.PI / 180.0))
        };

        public static final Pose2d[] RED_REEF_POSES = {
            new Pose2d(2.823 + 8.553921, 4.000, new Rotation2d(0 * Math.PI / 180.0)),
            new Pose2d(3.719 + 8.553921, 2.614, new Rotation2d(60 * Math.PI / 180.0)),
            new Pose2d(5.430 + 8.553921, 2.640, new Rotation2d(120 * Math.PI / 180.0)),
            new Pose2d(6.000 + 8.553921, 4.000, new Rotation2d(180 * Math.PI / 180.0)),
            new Pose2d(5.432 + 8.553921, 5.132, new Rotation2d(-120 * Math.PI / 180.0)),
            new Pose2d(3.689 + 8.553921, 5.515, new Rotation2d(-60 * Math.PI / 180.0))
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
        public static final int CORAL_RIGHT = 18;

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
