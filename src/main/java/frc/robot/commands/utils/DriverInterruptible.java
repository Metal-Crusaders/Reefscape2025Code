package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriverInterruptible extends SequentialCommandGroup {

    public DriverInterruptible(Command mainCommand, CommandXboxController xbox, double deadband) {
        // Create an InstantCommand that ends when the joystick is moved
        Command monitorJoystick = new RunCommand(() -> {
            // This can be expanded for finer control
            if (xbox.leftBumper().getAsBoolean()) {
                // The joystick has moved; the command will end
                this.cancel();
            }
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        // Combine the commands in a ParallelRaceGroup
        Command combined = new ParallelRaceGroup(
            mainCommand,     // The main command
            monitorJoystick  // Ends the group when joystick moves
        );

        // Add the combined command to this SequentialCommandGroup
        addCommands(combined);
    }
}
