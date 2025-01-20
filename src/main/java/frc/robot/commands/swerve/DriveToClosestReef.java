package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveToClosestReef extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;
    public DriveToClosestReef(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> (new ConditionalAllianceCommand(
                    this.swerve.driveToPose(MathUtils.findClosestTarget(this.swerve.getState().Pose, Constants.AutoDriveConstants.BLUE_REEF_POSES)),
                    this.swerve.driveToPose(MathUtils.findClosestTarget(this.swerve.getState().Pose, Constants.AutoDriveConstants.RED_REEF_POSES))
                )),
                getRequirements()
            )
        );
    }
}