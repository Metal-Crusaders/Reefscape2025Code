package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoDriveCoralRight extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoDriveCoralRight(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;
        
        // Define commands for each condition
        Command blueCmd = this.swerve.driveToPose(1.428, 0.828, 50);
        Command redCmd = this.swerve.driveToPose(16.394, 7.020, -125);

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