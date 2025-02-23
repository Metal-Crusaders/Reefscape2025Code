// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.TestPathAuto;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.elevator.ElevatorTeleop;
import frc.robot.commands.scoring.algae.AlgaeClawTeleop;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.DriveToClosestReef;
import frc.robot.commands.swerve.AutoDriveProcessor;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.utils.JoystickInterruptible;
import frc.robot.commands.coroutines.*;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.AlgaeClawConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.oi.MyButton;
import frc.robot.subsystems.camera.AprilTagCamera;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoring.AlgaeClaw;
import frc.robot.subsystems.scoring.AlgaePivot;
import frc.robot.subsystems.scoring.CoralShooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveTelemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    // OI STUFF
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // SUBSYSTEMS
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    public final Elevator elevator = new Elevator();
    public final CoralShooter coralShooter = new CoralShooter();
    public final AlgaeClaw algaeClaw = new AlgaeClaw();
    public final AlgaePivot algaePivot = new AlgaePivot();

    // CAMERA STUFF // TODO UNCOMMENT THIS WHEN PHOTONVISION STUFF COMES OUT
    public AprilTagCamera camera1 = new AprilTagCamera(Constants.CameraConstants.CAMERA_1_NAME, Constants.CameraConstants.CAMERA_1_POS, drivetrain);
    public AprilTagCamera camera2 = new AprilTagCamera(Constants.CameraConstants.CAMERA_2_NAME, Constants.CameraConstants.CAMERA_2_POS, drivetrain);

    // COMMANDS!!

    // drivetrain

    private final Command swerveTeleop = new SwerveTeleop(drivetrain, driverController);
    private final Command leftCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReef(drivetrain, 0), driverController, 0.5);
    private final Command rightCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReef(drivetrain, 1), driverController, 0.5);
    private final Command processorAutoDrive = new JoystickInterruptible(new AutoDriveProcessor(drivetrain), driverController, 0.5);
    private final Command reefAutoDrive = new JoystickInterruptible(new DriveToClosestReef(drivetrain), driverController, 0.5);

    // elevator commands
    // private final Command elevatorTeleop = new ElevatorTeleop(elevator, operatorController);
    private final ElevatorPreset restMode = new ElevatorPreset(elevator, Constants.ElevatorConstants.L1_ENCODER_TICKS);
    private final ElevatorPreset test1ElePreset = new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS);

    // coral shooter
    private final Command intakeCoral = new IntakeCoralFull(coralShooter);
    private final Command scoreCoral = new ScoreCoral(coralShooter);
    private final Command scoreCoralL1 = new ScoreCoralL1(coralShooter);

    // algae claw
    private final Command algaeClawTeleop = new AlgaeClawTeleop(algaeClaw, operatorController);
    private final Command algaeRestMode = new AlgaePivotPreset(algaePivot, AlgaeClawConstants.PIVOT_IN_TICKS);
    private final Command algaeClawOut = new AlgaePivotPreset(algaePivot, AlgaeClawConstants.PIVOT_OUT_TICKS);

    // coroutines
    // write code for all coroutines under the coroutines folder here:
    private final Command restModeCoroutine = new RestMode(elevator, algaePivot);
    private final Command lowAlgaeGrabCoroutine = new LowAlgaeGrab(drivetrain, elevator, coralShooter, algaePivot, algaeClaw);
    private final Command highAlgaeGrabCoroutine = new HighAlgaeGrab(drivetrain, elevator, coralShooter, algaePivot, algaeClaw);
    private final Command highAlgaeGrabCoralFirstCoroutine = new HighAlgaeGrabCoralFirst(drivetrain, elevator, coralShooter, algaePivot, algaeClaw);
    private final Command shootL1Coroutine = new ShootL1(drivetrain, elevator, coralShooter, algaePivot);
    private final Command shootL2LeftCoroutine = new ShootL2(false, drivetrain, elevator, coralShooter, algaePivot);
    private final Command shootL2RightCoroutine = new ShootL2(true, drivetrain, elevator, coralShooter, algaePivot);
    private final Command shootL3LeftCoroutine = new ShootL3(false, drivetrain, elevator, coralShooter, algaePivot);
    private final Command shootL3RightCoroutine = new ShootL3(false, drivetrain, elevator, coralShooter, algaePivot);
    private final Command processAlgaeCoroutine = new ProcessAlgae(algaePivot, algaeClaw);

    // autonomous
    // private final Command testPathAuto = new TestPathAuto(drivetrain);

    private void configureBindings() {

        operatorController.leftBumper().onTrue(intakeCoral);
        operatorController.rightBumper().onTrue(shootL1Coroutine);
        // operatorController.b().onTrue(scoreCoralL1);

        // driverController.leftBumper().whileTrue(leftCoralAutoDrive);
        // driverController.rightBumper().whileTrue(rightCoralAutoDrive);
        driverController.x().whileTrue(reefAutoDrive);
        driverController.b().onTrue(processorAutoDrive);

        drivetrain.setDefaultCommand(swerveTeleop);

        // algaeClaw.setDefaultCommand(algaeClawTeleop);
        // operatorController.x().onTrue(algaeClawOut);
        // operatorController.b().onTrue(algaeRestMode);

        // elevator.setDefaultCommand(elevatorTeleop);
        // operatorController.y().onTrue(test1ElePreset);
        // operatorController.a().onTrue(restMode);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        // return testPathAuto;
        return null;
    }
}
