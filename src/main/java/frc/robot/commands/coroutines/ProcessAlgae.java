package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
public class ProcessAlgae extends SequentialCommandGroup {
    
    public ProcessAlgae(Elevator elevator, AlgaePivot pivot, AlgaeClaw algaeClaw) {

        addRequirements(
            elevator,
            pivot,
            algaeClaw
        );

        // TODO add drivetrain alignment? and possibly elevator alignment
        addCommands(
            new ProcessAlgaeSubroutine(algaeClaw),
            new RestMode(elevator, pivot, algaeClaw)
        );

    }

}

