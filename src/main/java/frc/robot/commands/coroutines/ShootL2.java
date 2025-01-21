package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ShootL2 extends SequentialCommandGroup {
    
    public ShootL2(boolean right, CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter) {

        addCommands(
            // drive up (use conditional here)
            // raise elevator to correct height
            // spit out coral
            // return rest mode and back up at the same time
        );

    }

}
