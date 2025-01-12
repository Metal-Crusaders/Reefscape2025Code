package frc.robot.subsystems.camera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.Optional;

public class AprilTagCamera extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Timer timer;

    public AprilTagCamera(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {
        this.camera = new PhotonCamera(cameraName);
		aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // TODO THIS NEEDS CHANGING!!!
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        this.poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera
        );
        this.drivetrain = drivetrain;
        this.timer = new Timer();
        this.timer.start();
    }

        @Override
    public void periodic() {
        double timestamp = timer.get();

        // Get latest result from the camera
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("AprilTag Detected", true);
            SmartDashboard.putNumber("Target Count", result.targets.size());

            for (PhotonTrackedTarget target : result.targets) {
                SmartDashboard.putNumber("Target ID", target.getFiducialId());
                SmartDashboard.putNumber("Yaw", target.getYaw());
                SmartDashboard.putNumber("Pitch", target.getPitch());
                SmartDashboard.putNumber("Area", target.getArea());
            }

            // Pose estimation
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

            if (estimatedPose.isPresent()) {
                Pose3d visionPose = estimatedPose.get().estimatedPose;
                drivetrain.addVisionMeasurement(visionPose.toPose2d(), timestamp);
                SmartDashboard.putString("Vision Pose", visionPose.toString());
            }
        } else {
            SmartDashboard.putBoolean("AprilTag Detected", false);
        }
    }
}
