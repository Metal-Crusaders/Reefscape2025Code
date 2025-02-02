package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class IntakeCoral extends Command {

    private CoralShooter coralShooter;

    private final double INTAKE_SPEED_LEFT = -0.4;
    private final double INTAKE_SPEED_RIGHT = -0.2;

    public IntakeCoral(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;

    }

    @Override
    public void initialize() {
        coralShooter.stop();
    }
    

    @Override
    public void execute() {
        coralShooter.set(INTAKE_SPEED_LEFT, INTAKE_SPEED_RIGHT);
    }

    @Override
    public boolean isFinished() {
        return !this.coralShooter.beamExists();
    }

    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
    }

}
