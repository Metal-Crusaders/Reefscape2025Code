package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveToPoseUniversal extends SequentialCommandGroup {

    private static final double CLOSE_DRIVE_THRESHOLD = 0.25;

    private CommandSwerveDrivetrain swerve;
    public DriveToPoseUniversal(CommandSwerveDrivetrain swerve, Pose2d finalPose) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> (new ConditionalCommand(
                    new LongDriveToPose(swerve, finalPose),
                    new CloseDriveToPose(swerve, finalPose),
                    () -> finalPose.getTranslation().getDistance(swerve.getState().Pose.getTranslation()) > CLOSE_DRIVE_THRESHOLD
                )),
                getRequirements()
            )
        );
    }
    
}
