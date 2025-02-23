package frc.robot.commands.coroutines;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgaeTime;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.CloseDriveToClosestReef;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class HighAlgaeGrabCoralFirst extends SequentialCommandGroup {
    
    public HighAlgaeGrabCoralFirst(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot algaePivot, AlgaeClaw algaeClaw) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            coralShooter,
            algaeClaw,
            algaePivot
        );

        addCommands(
            new ParallelCommandGroup(
                new AutoLineUpReef(swerveDrivetrain, 0),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L3_ENCODER_TICKS)
            ),
            new ScoreCoral(coralShooter),
            new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS),
            new ParallelCommandGroup(
                new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
                new GrabAlgaeTime(algaeClaw, 2)
            ),
            new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_IN_ALGAE_TICKS),
            new ParallelCommandGroup(
                new CloseDriveToClosestReef(swerveDrivetrain),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L1_ENCODER_TICKS)
            )
        );
    }

}

