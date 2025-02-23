package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.AlgaeClaw;
import edu.wpi.first.wpilibj.Timer;

public class GrabAlgaeTime extends Command {

    private final AlgaeClaw claw;
    private final Timer timer = new Timer();
    
    private final double CLAW_INTAKE_SPEED = 0.5;
    private double seconds;

    public GrabAlgaeTime(AlgaeClaw claw, double seconds) {
        this.claw = claw;
        this.seconds = seconds;
        addRequirements(this.claw);
    }

    @Override
    public void initialize() {
        claw.setClawMotor(0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        claw.setClawMotor(CLAW_INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        claw.setClawMotor(0);
        timer.stop();
    }
}
