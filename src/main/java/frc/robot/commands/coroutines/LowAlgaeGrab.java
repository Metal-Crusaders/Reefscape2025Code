package frc.robot.commands.coroutines;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgae;
import frc.robot.commands.scoring.algae.GrabAlgaeTime;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestReef;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
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

public class LowAlgaeGrab extends SequentialCommandGroup {
    
    public LowAlgaeGrab(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot algaePivot, AlgaeClaw algaeClaw, CommandXboxController driverController) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            coralShooter,
            algaeClaw,
            algaePivot
        );

        addCommands(
            new ParallelCommandGroup(
                new CloseDriveToClosestReefGoodOffset(swerveDrivetrain),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.LOW_ALGAE_ENCODER_TICKS),
                new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS)
            ),
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerveDrivetrain, 0),
                new GrabAlgae(algaeClaw)
            ),
            new SwerveTeleopShortTerm(swerveDrivetrain, driverController),
            new ScoreCoral(coralShooter),
            new ParallelCommandGroup(
                new GrabAlgaeTime(algaeClaw, 2),
                new CloseDriveToClosestReefGoodOffset(swerveDrivetrain),
                new RestMode(elevator, algaePivot, algaeClaw)
            )
        );

    }

}

