package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorTeleop extends Command {

    private final Elevator elevator;
    private final CommandXboxController controller;

    public ElevatorTeleop(Elevator elevator, CommandXboxController operatorController) {
        this.elevator = elevator;
        this.controller = operatorController;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSpeed(0);
    }

    @Override
    public void execute() {

        double speed = controller.getLeftY();

        if ((elevator.getAvgEncoderTicks() >= ElevatorConstants.MAX_ENCODER_TICKS && speed < 0) || (elevator.getAvgEncoderTicks() <= 0 && speed > 0)) {
            speed = 0.0;
        }

        if (Math.abs(speed) < 0.15) {
            speed = 0.0;
        }

        speed *= -0.3;

        elevator.setSpeed(speed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0);
    }
    
}
