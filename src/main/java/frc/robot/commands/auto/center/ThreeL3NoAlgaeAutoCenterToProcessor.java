package frc.robot.commands.auto.center;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coroutines.LowAlgaeGrab;
import frc.robot.commands.coroutines.ProcessAlgae;
import frc.robot.commands.coroutines.ShootL3;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL3NoDriver;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.commands.scoring.coral.IntakeCoral;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ThreeL3NoAlgaeAutoCenterToProcessor extends SequentialCommandGroup {

    public ThreeL3NoAlgaeAutoCenterToProcessor(CommandSwerveDrivetrain swerve, Elevator elevator, AlgaeClaw algaeClaw, AlgaePivot algaePivot, CoralShooter coralShooter) {

        // drive from start to L3
        Command resetPose = new InstantCommand(), startToL3One, l3OneToCoralStation, l3ToCoralStation, coralStationToL3, coralStationToL32;
        Pose2d startingPose;
        try {
            startToL3One = swerve.driveAlongPath(PathPlannerPath.fromPathFile("StartToFirstL3"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("StartToFirstL3").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("StartToFirstL3").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            startToL3One = null; // or handle the error appropriately
        }
        try {
            l3OneToCoralStation = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3-1ToCoralStationProcessorSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3OneToCoralStation = null; // or handle the error appropriately
        }
        try {
            l3ToCoralStation = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3ToCoralStationProcessorSide"));
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
                startToL3One,
                new IntakeCoralFull(coralShooter)
            ),
            new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw),
            new ParallelCommandGroup(
                l3OneToCoralStation,
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new ProcessAlgae(elevator, algaePivot, algaeClaw)
                )
            ),
            new IntakeCoralFull(coralShooter), // coral station coroutine
            coralStationToL3,
            new ShootL3NoDriver(true, swerve, elevator, coralShooter, algaePivot, algaeClaw), // L3 coroutine
            l3ToCoralStation,
            new IntakeCoralFull(coralShooter), // coral station coroutine
            coralStationToL32,
            new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw) // low algae coroutine
        );

    }

}