package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class ScoreCoral extends Command {

    private CoralShooter coralShooter;

    private final double SHOOT_SPEED = 0.5;

    public ScoreCoral(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;

    }

    @Override
    public void initialize() {
        coralShooter.stop();
    }
    

    @Override
    public void execute() {
        coralShooter.set(SHOOT_SPEED, SHOOT_SPEED);
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
