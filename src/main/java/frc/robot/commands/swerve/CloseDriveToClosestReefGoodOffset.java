package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CloseDriveToClosestReefGoodOffset extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;
    public CloseDriveToClosestReefGoodOffset(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> (new ConditionalAllianceCommand(
                    new CloseDriveToPose(swerve, addRCtoFC(MathUtils.findClosestTarget(swerve.getState().Pose, Constants.AutoDriveConstants.BLUE_REEF_POSES))),
                    new CloseDriveToPose(swerve, addRCtoFC(MathUtils.findClosestTarget(swerve.getState().Pose, Constants.AutoDriveConstants.RED_REEF_POSES)))
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