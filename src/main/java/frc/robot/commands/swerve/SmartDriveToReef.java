package frc.robot.commands.swerve;

import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/**
 * TODO:
 * 1. close drive for all the close positions from coral station needs to be optimized
 */

public class SmartDriveToReef extends SequentialCommandGroup {

    // // reefPose can be a number from 0 through 5

    // private CommandSwerveDrivetrain swerve;
    // public SmartDriveToReef(CommandSwerveDrivetrain swerve, int reefPose) {

    //     this.swerve = swerve;

    //     addRequirements(this.swerve);

    //     // Add conditional logic to choose commands
    //     addCommands(
    //         new DeferredCommand(
    //             () -> new SequentialCommandGroup(
    //                 new ConditionalAllianceCommand(
    //                     new InstantCommand(() -> ReefStateManager.setTargetPose(addRCtoFC(Constants.AutoDriveConstants.BLUE_REEF_POSES[reefPose]))),
    //                     new InstantCommand(() -> ReefStateManager.setTargetPose(addRCtoFC(Constants.AutoDriveConstants.RED_REEF_POSES[reefPose])))
    //                 ),
    //                 new InstantCommand(
    //                     () -> ReefStateManager.setReefDistance(this.swerve.getState().Pose.getTranslation().getDistance(ReefStateManager.TARGET_POSE.getTranslation()))
    //                 ),
    //                 new InstantCommand(
    //                     () -> ReefStateManager.setPrevReefID(ReefStateManager.REEF_ID)
    //                 ),
    //                 new ConditionalAllianceCommand(
    //                     new InstantCommand(() -> ReefStateManager.setReefID(Constants.AutoDriveConstants.BLUE_REEF_IDS[reefPose])),
    //                     new InstantCommand(() -> ReefStateManager.setReefID(Constants.AutoDriveConstants.RED_REEF_IDS[reefPose]))
    //                 ),
    //                 new ConditionalAllianceCommand(
    //                     new InstantCommand(
    //                         () -> ReefStateManager.setLeftApproachDistance(this.swerve.getState().Pose.getTranslation().getDistance(Constants.AutoDriveConstants.BLUE_REEF_NEIGHBORS[reefPose][0].getTranslation()))
    //                     ),
    //                     new InstantCommand(
    //                         () -> ReefStateManager.setLeftApproachDistance(this.swerve.getState().Pose.getTranslation().getDistance(Constants.AutoDriveConstants.RED_REEF_NEIGHBORS[reefPose][0].getTranslation()))
    //                     )
    //                 ),
    //                 new ConditionalAllianceCommand(
    //                     new InstantCommand(
    //                         () -> ReefStateManager.setRightApproachDistance(this.swerve.getState().Pose.getTranslation().getDistance(Constants.AutoDriveConstants.BLUE_REEF_NEIGHBORS[reefPose][1].getTranslation()))
    //                     ),
    //                     new InstantCommand(
    //                         () -> ReefStateManager.setRightApproachDistance(this.swerve.getState().Pose.getTranslation().getDistance(Constants.AutoDriveConstants.RED_REEF_NEIGHBORS[reefPose][1].getTranslation()))
    //                     )
    //                 ),
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putString("Reef State Manager - Reef Pose", ReefStateManager.TARGET_POSE.toString())
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Reef ID", ReefStateManager.REEF_ID)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Prev Reef ID", ReefStateManager.PREV_REEF_ID)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Reef Distance", ReefStateManager.REEF_DISTANCE)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putBoolean("Reef State Manager - Approach Left", ReefStateManager.APPROACH_LEFT)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putBoolean("Reef State Manager - Driving Far", ReefStateManager.DRIVING_FAR)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Left Approach Distance", ReefStateManager.LEFT_APPROACH_DISTANCE)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Right Approach Distance", ReefStateManager.RIGHT_APPROACH_DISTANCE)
    //                 ),
    //                 new ConditionalCommand(
    //                     new SequentialCommandGroup(
    //                         new InstantCommand(() -> ReefStateManager.setDrivingFar(false)),
    //                         new InstantCommand(
    //                             () -> SmartDashboard.putString("DriveState", "CLOSE DRIVE PID")
    //                         ),
    //                         new CloseDriveToPose(swerve, ReefStateManager.TARGET_POSE)
    //                     ),
    //                     new ConditionalCommand(
    //                         new SequentialCommandGroup(
    //                             new InstantCommand(() -> ReefStateManager.setDrivingFar(false)),
    //                             new InstantCommand(
    //                                 () -> SmartDashboard.putString("DriveState", "CLOSE DRIVE PATHFINDING")
    //                             ),
    //                             swerve.driveToPose(ReefStateManager.TARGET_POSE)
    //                         ),
    //                         new ConditionalCommand(
    //                             new ConditionalCommand(
    //                                 new SequentialCommandGroup(
    //                                     new InstantCommand(() -> ReefStateManager.setApproachLeft(true)),
    //                                     new InstantCommand(() -> ReefStateManager.setDrivingFar(true)),
    //                                     new InstantCommand(
    //                                         () -> SmartDashboard.putString("DriveState", "LONG DRIVE APPROACH LEFT PRESSED BEFORE")
    //                                     ),
    //                                     swerve.reefDriveToPose(reefPose, true) // approach left, 
    //                                 ),
    //                                 new SequentialCommandGroup(
    //                                     new InstantCommand(() -> ReefStateManager.setApproachLeft(false)),
    //                                     new InstantCommand(() -> ReefStateManager.setDrivingFar(true)),
    //                                     new InstantCommand(
    //                                         () -> SmartDashboard.putString("DriveState", "LONG DRIVE APPROACH RIGHT PRESSED BEFORE")
    //                                     ),
    //                                     swerve.reefDriveToPose(reefPose, false) // approach right,
    //                                 ),
    //                                 () -> !ReefStateManager.APPROACH_LEFT
    //                             ),
    //                             new ConditionalCommand(
    //                                 new SequentialCommandGroup(
    //                                     new InstantCommand(() -> ReefStateManager.setApproachLeft(true)),
    //                                     new InstantCommand(() -> ReefStateManager.setDrivingFar(true)),
    //                                     new InstantCommand(
    //                                         () -> SmartDashboard.putString("DriveState", "LONG DRIVE APPROACH LEFT FIRST PRESS")
    //                                     ),
    //                                     swerve.reefDriveToPose(reefPose, true) // approach left, 
    //                                 ),
    //                                 new SequentialCommandGroup(
    //                                     new InstantCommand(() -> ReefStateManager.setApproachLeft(false)),
    //                                     new InstantCommand(() -> ReefStateManager.setDrivingFar(true)),
    //                                     new InstantCommand(
    //                                         () -> SmartDashboard.putString("DriveState", "LONG DRIVE APPROACH RIGHT FIRST PRESS")
    //                                     ),
    //                                     swerve.reefDriveToPose(reefPose, false) // approach left,
    //                                 ),
    //                                 () -> ReefStateManager.LEFT_APPROACH_DISTANCE < ReefStateManager.RIGHT_APPROACH_DISTANCE
    //                             ),
    //                             () -> ReefStateManager.PREV_REEF_ID == ReefStateManager.REEF_ID && ReefStateManager.DRIVING_FAR
    //                         ), 
    //                         () -> ReefStateManager.REEF_DISTANCE < ReefStateManager.RELATIVELY_CLOSE
    //                     ), 
    //                     () -> ReefStateManager.REEF_DISTANCE < ReefStateManager.UBER_CLOSE
    //                 ),
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putString("Reef State Manager - Reef Pose", ReefStateManager.TARGET_POSE.toString())
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Reef ID", ReefStateManager.REEF_ID)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Prev Reef ID", ReefStateManager.PREV_REEF_ID)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Reef Distance", ReefStateManager.REEF_DISTANCE)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putBoolean("Reef State Manager - Approach Left", ReefStateManager.APPROACH_LEFT)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putBoolean("Reef State Manager - Driving Far", ReefStateManager.DRIVING_FAR)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Left Approach Distance", ReefStateManager.LEFT_APPROACH_DISTANCE)
    //                 ),  
    //                 new InstantCommand(() -> 
    //                     SmartDashboard.putNumber("Reef State Manager - Right Approach Distance", ReefStateManager.RIGHT_APPROACH_DISTANCE)
    //                 )
    //             ),
    //             getRequirements()
    //         )
    //     );
    // }

    // public Pose2d addRCtoFC(Pose2d robotPose) {
    //     double xOffset = Constants.AutoDriveConstants.POSE_ADDITION[0];
    //     double yOffset = Constants.AutoDriveConstants.POSE_ADDITION[1];

    //     double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians()) - yOffset * Math.sin(robotPose.getRotation().getRadians()));
    //     double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians()) + yOffset * Math.cos(robotPose.getRotation().getRadians()));
    //     Pose2d calculated = new Pose2d(newX, newY, robotPose.getRotation());
    //     return calculated;
    // }
}