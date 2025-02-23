package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class ScoreCoral extends Command {

    private CoralShooter coralShooter;

    private final double SHOOT_SPEED = 0.8;
    private final double SHOOT_SECONDS = 0.5;
    private final Timer timer;

    public ScoreCoral(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        coralShooter.stop();
        timer.reset();
        timer.start();
    }
    

    @Override
    public void execute() {
        coralShooter.set(SHOOT_SPEED, SHOOT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return this.coralShooter.beamExists() && timer.hasElapsed(SHOOT_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
    }

}
