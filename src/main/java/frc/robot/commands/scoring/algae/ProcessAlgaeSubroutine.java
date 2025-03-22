package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.AlgaeClaw;

public class ProcessAlgaeSubroutine extends Command {
    
    private final AlgaeClaw claw;
    private final Timer timer;

    private final double CLAW_OUTTAKE_SPEED = 0.5;
    private final double TIMER_LENGTH = 0.5;

    public ProcessAlgaeSubroutine(AlgaeClaw claw) {
        this.claw = claw;
        this.timer = new Timer();

        addRequirements(this.claw);
    }

    @Override
    public void initialize() {
        claw.setClawMotor(0);
        claw.currentlyUsed = true;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        claw.setClawMotor(CLAW_OUTTAKE_SPEED);

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(TIMER_LENGTH);
    }

    @Override
    public void end(boolean interrupted) {
        claw.setClawMotor(0);
        claw.currentlyUsed = false;
    }
}
