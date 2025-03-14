package frc.robot.commands.auto.center;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coroutines.LowAlgaeGrab;
import frc.robot.commands.coroutines.ProcessAlgae;
import frc.robot.commands.coroutines.RestMode;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgae;
import frc.robot.commands.scoring.algae.GrabAlgaeTime;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestAlgaeOffset;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CenterL3AndDoubleProcessAuto extends SequentialCommandGroup {

    public CenterL3AndDoubleProcessAuto(CommandSwerveDrivetrain swerve, Elevator elevator, CoralShooter coralShooter, AlgaeClaw algaeClaw, AlgaePivot algaePivot) {

        Command resetPose = new InstantCommand(), startToL3, l3ToProcessor, processorToHighAlgae, highAlgaeToProcessor;
        Pose2d startingPose;

        try {
            startToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("StartToFirstL3Center"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("StartToFirstL3Center").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("StartToFirstL3Center").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            startToL3 = null; // or handle the error appropriately
        }
        try {
            l3ToProcessor = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3CenterToProcessor"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3ToProcessor = null; // or handle the error appropriately
        }
        try {
            processorToHighAlgae = swerve.driveAlongPath(PathPlannerPath.fromPathFile("ProcessorToHighAlgae"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            processorToHighAlgae = null; // or handle the error appropriately
        }
        try {
            highAlgaeToProcessor = swerve.driveAlongPath(PathPlannerPath.fromPathFile("HighAlgaeToProcessor"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            highAlgaeToProcessor = null; // or handle the error appropriately
        }


        addRequirements(swerve);

        addCommands(
            resetPose,
            new ParallelCommandGroup(
                new IntakeCoralFull(coralShooter),
                startToL3
            ),
            new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw),
            l3ToProcessor,
            new ProcessAlgae(elevator, algaePivot, algaeClaw),
            processorToHighAlgae,
            new ParallelCommandGroup(
                new CloseDriveToClosestReefGoodOffset(swerve),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS)
            ),
            new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
            new ParallelCommandGroup(
                new AutoLineUpReefUniversal(swerve, 0),
                new GrabAlgae(algaeClaw)
            ),
            new ParallelDeadlineGroup(
                new CloseDriveToClosestAlgaeOffset(swerve),
                new GrabAlgaeTime(algaeClaw, 0.5)
            ),
            new RestMode(elevator, algaePivot, algaeClaw),
            highAlgaeToProcessor,
            new ProcessAlgae(elevator, algaePivot, algaeClaw)
        );

    }

}