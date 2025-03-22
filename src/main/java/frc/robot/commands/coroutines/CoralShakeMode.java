package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CoralShakeMode extends SequentialCommandGroup {
    
    public CoralShakeMode(Elevator elevator, AlgaePivot algaePivot, AlgaeClaw algaeClaw) {

        addRequirements(
            elevator,
            algaePivot
        );

        double OFFSET = 6;

        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
                    new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS + OFFSET)
                ),
                new SequentialCommandGroup(
                    new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_IN_TICKS),
                    new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS + OFFSET)
                ),
                () -> algaeClaw.holdingAlgae()
            )
        );

    }

}

