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

public class HighAlgaeGrab extends SequentialCommandGroup {
    
    public HighAlgaeGrab(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot algaePivot, AlgaeClaw algaeClaw) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            coralShooter,
            algaeClaw,
            algaePivot
        );

        addCommands(
            new ParallelCommandGroup(
                new CloseDriveToClosestReef(swerveDrivetrain),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS),
                new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS)
            ),
            new ParallelCommandGroup(
                new AutoLineUpReef(swerveDrivetrain, 0),
                new GrabAlgaeTime(algaeClaw, 2)
            ),
            new CloseDriveToClosestReef(swerveDrivetrain),
            new ElevatorPreset(elevator, Constants.ElevatorConstants.L3_ENCODER_TICKS),
            new AutoLineUpReef(swerveDrivetrain, 0),
            new ScoreCoral(coralShooter),
            new ParallelCommandGroup(
                new CloseDriveToClosestReef(swerveDrivetrain),
                new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_IN_ALGAE_TICKS)
            ),
            new ElevatorPreset(elevator, Constants.ElevatorConstants.L1_ENCODER_TICKS)
        );
    }

}

