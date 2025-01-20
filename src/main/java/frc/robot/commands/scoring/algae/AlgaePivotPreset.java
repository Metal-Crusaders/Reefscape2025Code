package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.scoring.AlgaeClaw;

public class AlgaePivotPreset extends Command {

    private final AlgaeClaw claw;
    private final double encoderTicks;

    private boolean finishedSetting;

    public AlgaePivotPreset(AlgaeClaw claw, double encoderTicks) {

        this.claw = claw;
        this.encoderTicks = encoderTicks;
        this.finishedSetting = false;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        
        finishedSetting = false;

        claw.setPivotMotor(encoderTicks);

        finishedSetting = true;
        
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("Algae Encoder Ticks", claw.getPivotEncoderTicks());
        
    }

    @Override
    public boolean isFinished() {
        return finishedSetting && Math.abs(this.claw.getPivotEncoderTicks() - encoderTicks) < Constants.AlgaeClawConstants.PIVOT_TICKS_DEADBAND;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
