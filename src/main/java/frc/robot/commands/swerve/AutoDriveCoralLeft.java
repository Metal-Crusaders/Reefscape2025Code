package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoDriveCoralLeft extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoDriveCoralLeft(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;
        
        // Define commands for each condition
        Command blueCmd = this.swerve.driveToPose(1.142, 7.007, -50);
        Command redCmd = this.swerve.driveToPose(16.461, 1.134, 125);

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new ConditionalAllianceCommand(
                blueCmd,
                redCmd
            )
        );
    }
}