package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ShootL1 extends SequentialCommandGroup {

    public ShootL1(CommandSwerveDrivetrain swerveDrivetrain, CoralShooter coralShooter) {

        addCommands(
            // drive up
            // spit out coral
        );

    }
    
}
