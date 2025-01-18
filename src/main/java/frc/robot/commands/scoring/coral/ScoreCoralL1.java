package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class ScoreCoralL1 extends Command {

    private CoralShooter coralShooter;

    private final double SHOOT_SPEED_LEFT = 0.5;
    private final double SHOOT_SPEED_RIGHT = 0.5;

    public ScoreCoralL1(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;

    }

    @Override
    public void initialize() {
        coralShooter.stop();
    }
    

    @Override
    public void execute() {
        coralShooter.set(SHOOT_SPEED_LEFT, SHOOT_SPEED_RIGHT);
    }

    @Override
    public boolean isFinished() {
        return this.coralShooter.beamExists();
    }

    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
    }

}
