package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.CoralShooter;

public class SmallTurn extends Command {

    private CoralShooter coralShooter;

    private final double INTAKE_SPEED = 0.4;

    private final double NUM_TICKS_ROTATE = 2700;

    public SmallTurn(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;

    }

    @Override
    public void initialize() {
        coralShooter.stop();
        coralShooter.resetEncoders();
    }
    

    @Override
    public void execute() {
        SmartDashboard.putNumber("Coral Shooter Encoder Ticks", this.coralShooter.getAvgEncoderTicks());
        coralShooter.set(INTAKE_SPEED, INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return this.coralShooter.getAvgEncoderTicks() >= NUM_TICKS_ROTATE;
    }

    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
    }

}
