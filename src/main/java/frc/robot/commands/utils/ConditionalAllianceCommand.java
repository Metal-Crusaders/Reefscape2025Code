package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class ConditionalAllianceCommand extends SequentialCommandGroup {

    public ConditionalAllianceCommand(Command blueCmd, Command redCmd) {

        // Add conditional logic to choose commands
        addCommands(
            new ConditionalCommand(
                blueCmd,
                redCmd,
                () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
            )
        );
    }
}