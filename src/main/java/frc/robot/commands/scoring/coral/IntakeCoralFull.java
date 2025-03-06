package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.scoring.CoralShooter;

public class IntakeCoralFull extends SequentialCommandGroup {

    private final CoralShooter coralShooter;

    public IntakeCoralFull(CoralShooter coralShooter) {

        this.coralShooter = coralShooter;

        addRequirements(this.coralShooter);

        addCommands(
            new IntakeCoral(this.coralShooter)
            // new WaitCommand(1),
            // new SmallTurn(this.coralShooter)
        );

    }
    
}
