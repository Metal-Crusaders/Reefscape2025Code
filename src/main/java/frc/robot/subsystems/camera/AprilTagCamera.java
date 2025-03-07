package frc.robot.subsystems.camera;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.Optional;

public class AprilTagCamera extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final SendableChooser<Boolean> cameraEnabled;

    public AprilTagCamera(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {
        this.camera = new PhotonCamera(cameraName);
        aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        this.poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera
        );
        this.drivetrain = drivetrain;

        cameraEnabled = new SendableChooser<Boolean>();
        cameraEnabled.setDefaultOption(cameraName + "/Enabled", true);
        cameraEnabled.addOption("Disabled", false);
        SmartDashboard.putData(String.format("%s Enabled?", cameraName), cameraEnabled);

    }

    @Override
    public void periodic() {

        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {

            SmartDashboard.putBoolean("AprilTag Detected", result.hasTargets());

            boolean hasPose = false;

            if (result.hasTargets()) {
                SmartDashboard.putNumber(cameraName + "/Target Count", result.targets.size());

                for (PhotonTrackedTarget target : result.targets) {
                    SmartDashboard.putNumber(cameraName + "/Target ID", target.getFiducialId());
                    SmartDashboard.putNumber(cameraName + "/Yaw", target.getYaw());
                    SmartDashboard.putNumber(cameraName + "/Pitch", target.getPitch());
                    SmartDashboard.putNumber(cameraName + "/Area", target.getArea());

                    for (int tag : Constants.AutoDriveConstants.GOOD_APRIL_TAGS) {
                        if (target.getFiducialId() == tag) {
                            hasPose = true;
                            break;
                        }
                    }
                    
                    if (hasPose) {
                        break;
                    }
                }

                if (hasPose) {
                    // Pose estimation
                    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

                    if (estimatedPose.isPresent()) {
                        Pose3d visionPose = estimatedPose.get().estimatedPose;
                        if (this.cameraEnabled.getSelected()) {
                            drivetrain.addVisionMeasurement(visionPose.toPose2d(), Utils.getCurrentTimeSeconds());
                        }
                        SmartDashboard.putString(cameraName + "/Vision Pose", visionPose.toString());
                    }
                }
            } else {
            }
        }
    }
}
