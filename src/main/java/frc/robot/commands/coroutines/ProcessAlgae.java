package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ProcessAlgae extends SequentialCommandGroup {
    
    public ProcessAlgae(CommandSwerveDrivetrain swerveDrivetrain, AlgaeClaw algaeClaw) {

        addCommands(
            // drive up
            // pivot out and spit algae until spit (SAME TIME)
            // pivot in and back up (SAME TIME)
        );

    }

}

