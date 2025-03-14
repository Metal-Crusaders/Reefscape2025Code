package frc.robot.commands.auto.processorSide;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coroutines.LowAlgaeGrab;
import frc.robot.commands.coroutines.ProcessAlgae;
import frc.robot.commands.coroutines.RestMode;
import frc.robot.commands.coroutines.ShootL1;
import frc.robot.commands.coroutines.ShootL3;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL1NoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL3NoDriver;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgae;
import frc.robot.commands.scoring.algae.GrabAlgaeTime;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.commands.scoring.coral.IntakeCoral;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestAlgaeOffset;
import frc.robot.commands.swerve.CloseDriveToClosestReef;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class L2AndTwoL3AutoProcessorSide extends SequentialCommandGroup {

    public L2AndTwoL3AutoProcessorSide(CommandSwerveDrivetrain swerve, Elevator elevator, AlgaeClaw algaeClaw, AlgaePivot algaePivot, CoralShooter coralShooter) {

        // drive from start to L3
        Command resetPose = new InstantCommand(), startToL2, l2ToCoralStation, coralStationToL3, l3ToCoralStation, coralStationToL32;
        Pose2d startingPose;
        try {
            startToL2 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("StartingToL2ProcessorSide"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("StartingToL2ProcessorSide").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("StartingToL2ProcessorSide").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            startToL2 = null; // or handle the error appropriately
        }
        try {
            l2ToCoralStation = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L2ToCoralStationProcessorSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l2ToCoralStation = null; // or handle the error appropriately
        }
        try {
            l3ToCoralStation = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3ToCoralStationProcessorSideWithAlgae"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3ToCoralStation = null; // or handle the error appropriately
        }
        try {
            coralStationToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CoralStationToL3ProcessorSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            coralStationToL3 = null; // or handle the error appropriately
        }
        try {
            coralStationToL32 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CoralStationToL3ProcessorSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            coralStationToL32 = null; // or handle the error appropriately
        }

        addRequirements(
            swerve,
            elevator,
            algaeClaw,
            algaePivot,
            coralShooter
        );

        addCommands(
            resetPose,
            new ParallelCommandGroup(
                startToL2,
                new IntakeCoralFull(coralShooter)
            ),
            // L3 coroutine
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerve, 1),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS)
            ),
            new ScoreCoral(coralShooter),
            // l3 to coral station
            new ParallelCommandGroup(
                new RestMode(elevator, algaePivot, algaeClaw),
                l2ToCoralStation
            ),
            new IntakeCoralFull(coralShooter), // coral station coroutine
            // low algae coroutine
            new ParallelCommandGroup(
                coralStationToL3,
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new ElevatorPreset(elevator, Constants.ElevatorConstants.LOW_ALGAE_ENCODER_TICKS),
                    new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS)
                )
            ),
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerve, 0),
                new GrabAlgae(algaeClaw)
            ),
            new ScoreCoral(coralShooter),
            // rest of the path
            new ParallelCommandGroup(
                l3ToCoralStation,
                new SequentialCommandGroup(
                    new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
                    new ElevatorPreset(elevator, Constants.ElevatorConstants.PROCESSOR_ALGAE_TICKS),
                    new WaitCommand(0.2),
                    new ProcessAlgae(elevator, algaePivot, algaeClaw)
                )
            ),
            new IntakeCoralFull(coralShooter),
            new ParallelCommandGroup(
                coralStationToL32,
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L3_ENCODER_TICKS)
            ),
            new AutoLineUpReefUniversal(swerve, 0),
            new ScoreCoralL1(coralShooter),
            new ParallelCommandGroup(
                new CloseDriveToClosestReefGoodOffset(swerve),
                new RestMode(elevator, algaePivot, algaeClaw)
            )
        );

    }

}