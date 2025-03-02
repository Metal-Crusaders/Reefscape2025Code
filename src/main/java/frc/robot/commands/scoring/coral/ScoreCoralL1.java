package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class ScoreCoralL1 extends Command {

    private CoralShooter coralShooter;

    private final double SHOOT_SPEED_LEFT = 0.4;
    private final double SHOOT_SECONDS = 0.5;
    private final Timer timer;
    private final double SHOOT_SPEED_RIGHT = 0.15;

    public ScoreCoralL1(CoralShooter coralShooter) {

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
        coralShooter.set(SHOOT_SPEED_LEFT, SHOOT_SPEED_RIGHT);
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
