package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class HighAlgaeGrab extends SequentialCommandGroup {
    
    public HighAlgaeGrab(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaeClaw algaeClaw) {

        addCommands(
            // drive up
            // raise elevator to correct height
            // pivot out and grab algae until grabbed (SAME TIME)
            // pivot in and lower elevator height (SAME TIME)
            // spit out coral until spit
            // return rest mode and back up (SAME TIME)
        );

    }

}

