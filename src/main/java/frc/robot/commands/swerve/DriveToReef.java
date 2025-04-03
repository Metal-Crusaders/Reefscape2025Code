package frc.robot.commands.swerve;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.commands.utils.DisplayImageCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.util.ReefAlignmentPID;
import frc.robot.subsystems.util.ReefDisplay;

public class DriveToReef extends SequentialCommandGroup {

    // reefPose can be a number from 0 through 5

    private CommandSwerveDrivetrain swerve;
    public DriveToReef(CommandSwerveDrivetrain swerve, ReefDisplay reefDisplay, ReefAlignmentPID reefAlignmentPID, int reefPose) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> (new SequentialCommandGroup(
                    new DisplayImageCommand(reefDisplay, reefPose),
                    new InstantCommand(
                        () -> PPHolonomicDriveController.overrideRotationFeedback(reefAlignmentPID::getPIDCalculation)
                    ),
                    new ConditionalAllianceCommand(
                        new DriveToPoseUniversal(swerve, addRCtoFC(Constants.AutoDriveConstants.BLUE_REEF_POSES[reefPose])),
                        new DriveToPoseUniversal(swerve, addRCtoFC(Constants.AutoDriveConstants.RED_REEF_POSES[reefPose]))
                    ),
                    new InstantCommand(
                        () -> PPHolonomicDriveController.clearFeedbackOverrides()
                    )
                )),
                getRequirements()
            )
        );
    }

    public Pose2d addRCtoFC(Pose2d robotPose) {
        double xOffset = Constants.AutoDriveConstants.POSE_ADDITION[0];
        double yOffset = Constants.AutoDriveConstants.POSE_ADDITION[1];

        double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians()) - yOffset * Math.sin(robotPose.getRotation().getRadians()));
        double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians()) + yOffset * Math.cos(robotPose.getRotation().getRadians()));
        Pose2d calculated = new Pose2d(newX, newY, robotPose.getRotation());
        return calculated;
    }
}