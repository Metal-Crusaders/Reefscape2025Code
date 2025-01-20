package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.scoring.AlgaeClaw;

public class AlgaeClawTeleop extends Command {

    private final AlgaeClaw claw;
    private final CommandXboxController controller;

    public AlgaeClawTeleop(AlgaeClaw claw, CommandXboxController operatorController) {
        this.claw = claw;
        this.controller = operatorController;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setClawMotor(0);
        claw.setPivotMotor(0);
    }

    @Override
    public void execute() {

        double clawSpeed = controller.getLeftY();
        double pivotSpeed = controller.getRightY();

        claw.setClawMotor(clawSpeed);
        claw.setPivotMotor(pivotSpeed);

        SmartDashboard.putNumber("Algae Claw Current", claw.getClawMotor().getOutputCurrent()); // TODO use this to identify the current max for deadbanding
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setClawMotor(0);
        claw.setPivotMotor(0);
    }
    
}
