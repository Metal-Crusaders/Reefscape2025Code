package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.AlgaeClaw;

public class GrabAlgae extends Command {

    private final AlgaeClaw claw;

    private final double CLAW_INTAKE_SPEED = -0.6;

    public GrabAlgae(AlgaeClaw claw) {
        this.claw = claw;

        addRequirements(this.claw);
    }

    @Override
    public void initialize() {
        claw.currentlyUsed = true;
        claw.setClawMotor(0);
    }

    @Override
    public void execute() {

        claw.setClawMotor(CLAW_INTAKE_SPEED);

    }

    @Override
    public boolean isFinished() {
        return claw.holdingAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        claw.currentlyUsed = false;
        claw.setClawMotor(0);
    }
    
    
}
