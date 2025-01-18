// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.TestPathAuto;
import frc.robot.commands.scoring.coral.IntakeCoral;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.camera.AprilTagCamera;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveTelemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // SUBSYSTEMS
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    // public final CoralShooter coralShooter = new CoralShooter();

    // CAMERA STUFF // TODO UNCOMMENT THIS WHEN PHOTONVISION STUFF COMES OUT
    public AprilTagCamera camera = new AprilTagCamera(Constants.CameraConstants.CAMERA_1_NAME, Constants.CameraConstants.CAMERA_1_POS, drivetrain);

    // COMMANDS!!

    // drivetrain

    private final Command swerveTeleop = new SwerveTeleop(drivetrain, driverController);
    private final Command topCoralAutoDrive = drivetrain.driveToPose(1.020, 6.922, 130, driverController);
    private final Command bottomCoralAutoDrive = drivetrain.driveToPose(1.428, 0.828, -130, driverController);

    // shooting test
    // private final Command intakeCoral = new IntakeCoral(coralShooter);
    // private final Command scoreCoral = new ScoreCoral(coralShooter);
    // private final Command scoreCoralL1 = new ScoreCoralL1(coralShooter);

    // autonomous
    private final Command testPathAuto = new TestPathAuto(drivetrain);

    private void configureBindings() {
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // operatorController.a().onTrue(intakeCoral);
        // operatorController.x().onTrue(scoreCoral);
        // operatorController.b().onTrue(scoreCoralL1);

        driverController.leftBumper().onTrue(topCoralAutoDrive);
        driverController.rightBumper().onTrue(bottomCoralAutoDrive);

        drivetrain.setDefaultCommand(swerveTeleop);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return testPathAuto;
    }
}
