package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoDriveProcessor extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoDriveProcessor(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;
        
        // Define commands for each condition
        Command blueCmd = this.swerve.driveToPose(6.034, 0.578, -90);
        Command redCmd = this.swerve.driveToPose(11.412, 7.486, 90);

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