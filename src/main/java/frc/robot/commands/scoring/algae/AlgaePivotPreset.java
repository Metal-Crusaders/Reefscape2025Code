package frc.robot.commands.scoring.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlgaeClawConstants;
import frc.robot.subsystems.scoring.AlgaePivot;

public class AlgaePivotPreset extends Command {

    private final AlgaePivot claw;
    private final double encoderTicks;

    private boolean finishedSetting;

    public AlgaePivotPreset(AlgaePivot claw, double encoderTicks) {

        this.claw = claw;
        this.encoderTicks = encoderTicks;
        this.finishedSetting = false;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        
        finishedSetting = false;

        claw.setPivotPosition(encoderTicks);

        finishedSetting = true;
        
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("Algae Encoder Ticks", claw.getPivotEncoderTicks());
        SmartDashboard.putNumber("Algae Pivot", claw.getPivotMotor().get());
        SmartDashboard.putNumber("Algae Pivot Applied", claw.getPivotMotor().getAppliedOutput());
        
    }

    @Override
    public boolean isFinished() {
        return finishedSetting && Math.abs(this.claw.getPivotEncoderTicks() - encoderTicks) < Constants.AlgaeClawConstants.PIVOT_TICKS_DEADBAND;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
