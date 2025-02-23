package frc.robot.commands.coroutines;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;

public class ShootL1 extends SequentialCommandGroup {

    public ShootL1(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot pivot) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            pivot,
            coralShooter
        );

        addCommands(
            new AutoLineUpReef(swerveDrivetrain, 0),
            new RestMode(elevator, pivot),
            new ScoreCoralL1(coralShooter)
        );

    }
    
}
