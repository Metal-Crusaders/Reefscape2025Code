package frc.robot.commands.coroutines.nodriver;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.commands.coroutines.RestMode;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;

public class ShootL1NoDriver extends SequentialCommandGroup {

    public ShootL1NoDriver(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, CoralShooter coralShooter, AlgaePivot pivot, AlgaeClaw claw) {

        addRequirements(
            swerveDrivetrain,
            elevator,
            pivot,
            coralShooter,
            claw
        );

        addCommands(
            new AutoLineUpReefUniversal(swerveDrivetrain, 0),
            new RestMode(elevator, pivot, claw),
            new ScoreCoralL1(coralShooter)
        );

    }
    
}
