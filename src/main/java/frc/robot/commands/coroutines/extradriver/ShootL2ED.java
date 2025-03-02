package frc.robot.commands.coroutines.extradriver;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.coroutines.RestMode;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
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

public class ShootL2ED extends SequentialCommandGroup {
    
    public ShootL2ED(boolean right, CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot pivot, AlgaeClaw claw, CommandXboxController driverController) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            pivot,
            coralShooter,
            claw
        );

        addCommands(
            // new AutoLineUpReefUniversal(swerveDrivetrain, (right ? 1 : 0)),
            new ParallelCommandGroup(
                new SwerveTeleopShortTerm(swerveDrivetrain, driverController),
                new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS)
            ),
            new ScoreCoral(coralShooter),
            new ParallelCommandGroup(
                new RestMode(elevator, pivot, claw),
                new CloseDriveToClosestReef(swerveDrivetrain)
            )
        );

    }

}
