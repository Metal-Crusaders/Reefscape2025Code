package frc.robot.commands.coroutines.nodriver;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.coroutines.RestMode;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgae;
import frc.robot.commands.scoring.algae.GrabAlgaeTime;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestAlgaeOffset;
import frc.robot.commands.swerve.CloseDriveToClosestReef;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.swerve.SwerveTeleopShortTerm;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class HighAlgaeGrabCoralFirstNoDriver extends SequentialCommandGroup {
    
    public HighAlgaeGrabCoralFirstNoDriver(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot algaePivot, AlgaeClaw algaeClaw) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            coralShooter,
            algaeClaw,
            algaePivot
        );

        addCommands(
            new CloseDriveToClosestReefGoodOffset(swerveDrivetrain),
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerveDrivetrain, 0),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS) 
            ),
            new ScoreCoral(coralShooter),
            new ParallelCommandGroup(
                new CloseDriveToClosestReefGoodOffset(swerveDrivetrain),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS)
            ),
            new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerveDrivetrain, 0),
                new GrabAlgae(algaeClaw)
            ),
            new ParallelCommandGroup(
                new GrabAlgaeTime(algaeClaw, 0.5),
                new CloseDriveToClosestAlgaeOffset(swerveDrivetrain)
            ),
            new RestMode(elevator, algaePivot, algaeClaw)
        );
    }

}

