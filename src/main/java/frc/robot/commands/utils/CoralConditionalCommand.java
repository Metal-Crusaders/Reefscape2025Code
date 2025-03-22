package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.CoralShooter;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CoralConditionalCommand extends SequentialCommandGroup {

    public CoralConditionalCommand(Command cmd, CoralShooter coralShooter) {

        // Add conditional logic to choose commands
        addCommands(
            new ConditionalCommand(
                cmd,
                new InstantCommand(),
                () -> !coralShooter.beamExists()
            )
        );
    }
}