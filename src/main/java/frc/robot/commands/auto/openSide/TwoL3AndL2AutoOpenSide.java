package frc.robot.commands.auto.openSide;

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
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TwoL3AndL2AutoOpenSide extends SequentialCommandGroup {

    public TwoL3AndL2AutoOpenSide(CommandSwerveDrivetrain swerve, Elevator elevator, AlgaeClaw algaeClaw, AlgaePivot algaePivot, CoralShooter coralShooter) {

        // drive from start to L3
        Command resetPose = new InstantCommand(), startToL3, l3ToCoralStation, coralStationToL3, l3ToCoralStation2, coralStationToL2;
        Pose2d startingPose;
        try {
            startToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("StartingToL3OpenSide"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("StartingToL3OpenSide").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("StartingToL3OpenSide").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            startToL3 = null; // or handle the error appropriately
        }
        try {
            l3ToCoralStation = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3ToCoralStationOpenSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3ToCoralStation = null; // or handle the error appropriately
        }
        try {
            l3ToCoralStation2 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3ToCoralStationOpenSideWithAlgae"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3ToCoralStation2 = null; // or handle the error appropriately
        }
        try {
            coralStationToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CoralStationToL3OpenSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            coralStationToL3 = null; // or handle the error appropriately
        }
        try {
            coralStationToL2 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CoralStationToL2OpenSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            coralStationToL2 = null; // or handle the error appropriately
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
                startToL3,
                new IntakeCoralFull(coralShooter)
            ),
            // L3 coroutine
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerve, 1),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L3_ENCODER_TICKS)
            ),
            new ScoreCoral(coralShooter),
            // l3 to coral station
            new ParallelCommandGroup(
                new RestMode(elevator, algaePivot, algaeClaw),
                l3ToCoralStation
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
                l3ToCoralStation2,
                new SequentialCommandGroup(
                    new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
                    new ElevatorPreset(elevator, Constants.ElevatorConstants.PROCESSOR_ALGAE_TICKS),
                    new WaitCommand(0.2),
                    new ProcessAlgae(elevator, algaePivot, algaeClaw)
                )
            ),
            new IntakeCoralFull(coralShooter),
            
            new ParallelCommandGroup(
                coralStationToL2,
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS),
                new SequentialCommandGroup(
                    new WaitCommand(1.25),
                    new ScoreCoralL1(coralShooter)
                )
            ),
            // new AutoLineUpReefUniversal(swerve, 0),
            new RestMode(elevator, algaePivot, algaeClaw)
        );

    }

}