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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coroutines.LowAlgaeGrab;
import frc.robot.commands.coroutines.ProcessAlgae;
import frc.robot.commands.coroutines.ShootL3;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL3NoDriver;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.commands.scoring.coral.IntakeCoral;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TwoL3BuddyAutoOpenSide extends SequentialCommandGroup {

    public TwoL3BuddyAutoOpenSide(CommandSwerveDrivetrain swerve, Elevator elevator, AlgaeClaw algaeClaw, AlgaePivot algaePivot, CoralShooter coralShooter) {

        // drive from start to L3
        Command resetPose = new InstantCommand(), buddyBump, startToL3, l3ToCoralStation, coralStationToL3, l3ToBump;
        Pose2d startingPose;
        try {
            buddyBump = swerve.driveAlongPath(PathPlannerPath.fromPathFile("BuddyBumpOpenSide"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("BuddyBumpOpenSide").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("BuddyBumpOpenSide").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            buddyBump = null; // or handle the error appropriately
        }
        try {
            startToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("StartingToL3OpenSide"));
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
            coralStationToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CoralStationToL3OpenSide"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            coralStationToL3 = null; // or handle the error appropriately
        }
        try {
            l3ToBump = swerve.driveAlongPath(PathPlannerPath.fromPathFile("L3ToBumpDisengageAlgae"));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            l3ToBump = null; // or handle the error appropriately
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
                new SequentialCommandGroup(
                    buddyBump,
                    startToL3
                ),
                new IntakeCoralFull(coralShooter)
            ),
            new ShootL3NoDriver(true, swerve, elevator, coralShooter, algaePivot, algaeClaw), // L3 coroutine
            l3ToCoralStation,
            new IntakeCoralFull(coralShooter), // coral station coroutine
            coralStationToL3,
            new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw), // low algae coroutine
            new ParallelCommandGroup(
                l3ToBump,
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new ProcessAlgae(elevator, algaePivot, algaeClaw)
                )
            )
        );

    }

}