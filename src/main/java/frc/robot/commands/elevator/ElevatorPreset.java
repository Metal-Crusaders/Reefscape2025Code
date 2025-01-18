package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorPreset extends Command {

    private final Elevator elevator;
    private final double encoderTicks;

    private boolean finishedSetting;

    public ElevatorPreset(Elevator elevator, double encoderTicks) {

        this.elevator = elevator;
        this.encoderTicks = encoderTicks;
        this.finishedSetting = false;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
        finishedSetting = false;

        elevator.setElevatorPosition(encoderTicks);

        finishedSetting = true;
        
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("Elevator Encoder Ticks", elevator.getAvgEncoderTicks());
        
    }

    @Override
    public boolean isFinished() {
        return finishedSetting && Math.abs(this.elevator.getAvgEncoderTicks() - encoderTicks) < Constants.ElevatorConstants.ELEVATOR_TICKS_DEADBAND;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
