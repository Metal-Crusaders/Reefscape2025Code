package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class WaitUntilB extends Command {
    private final CommandXboxController controller;

    public WaitUntilB(CommandXboxController controller) {
        this.controller = controller;
        addRequirements(); // Add requirements here if necessary, or leave empty
    }

    @Override
    public void initialize() {
        // You can perform any initialization you need here
    }

    @Override
    public void execute() {
        // Check if the X button is pressed
        if (controller.b().getAsBoolean()) {
            // Immediately end the command if the X button is pressed
            this.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        // The command finishes as soon as the X button is pressed
        return controller.b().getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally handle what happens when the command ends
        // You can log, stop motors, or clean up if needed
    }
}
