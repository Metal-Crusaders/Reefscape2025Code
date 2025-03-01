package frc.robot.commands.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coroutines.LowAlgaeGrab;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TestAlgaeL3Auto extends SequentialCommandGroup {

    public TestAlgaeL3Auto(CommandSwerveDrivetrain swerve, Elevator elevator, CoralShooter coralShooter, AlgaeClaw algaeClaw, AlgaePivot algaePivot) {

        Command resetPose = new InstantCommand(), startToL3;
        Pose2d startingPose;

        try {
            startToL3 = swerve.driveAlongPath(PathPlannerPath.fromPathFile("TestPath"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("TestPath").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("TestPath").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            startToL3 = null; // or handle the error appropriately
        }

        addRequirements(swerve);

        addCommands(
            resetPose,
            new ParallelCommandGroup(
                new IntakeCoralFull(coralShooter),
                startToL3
            ),
            new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw)
        );

    }

}