package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RestMode extends SequentialCommandGroup {
    
    public RestMode(Elevator elevator, AlgaePivot algaeClaw) {

        addRequirements(
            elevator,
            algaeClaw
        );

        addCommands(
            new AlgaePivotPreset(algaeClaw, Constants.AlgaeClawConstants.PIVOT_IN_ALGAE_TICKS),
            new ElevatorPreset(elevator, Constants.ElevatorConstants.L1_ENCODER_TICKS)
        );

    }

}

