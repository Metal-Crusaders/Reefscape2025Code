package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class LongDriveToPose extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;
    public LongDriveToPose(CommandSwerveDrivetrain swerve, Pose2d finalPose) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> this.swerve.driveToPose(finalPose),
                getRequirements()
            )
        );
    }
}