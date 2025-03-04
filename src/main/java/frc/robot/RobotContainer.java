// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.elevator.ElevatorTeleop;
import frc.robot.commands.scoring.algae.AlgaeClawTeleop;
import frc.robot.commands.scoring.algae.AlgaePivotPreset;
import frc.robot.commands.scoring.algae.GrabAlgae;
import frc.robot.commands.scoring.algae.ProcessAlgaeSubroutine;
import frc.robot.commands.scoring.coral.IntakeCoralFull;
import frc.robot.commands.scoring.coral.ScoreCoral;
import frc.robot.commands.scoring.coral.ScoreCoralL1;
import frc.robot.commands.swerve.DriveToClosestReef;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.swerve.AutoDriveProcessor;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestReef;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.swerve.SwerveTeleopShortTerm;
import frc.robot.commands.utils.JoystickInterruptible;
import frc.robot.commands.auto.TestAlgaeL3Auto;
import frc.robot.commands.auto.TwoL3AlgaeAuto;
import frc.robot.commands.auto.TwoL3NoAlgaeAuto;
import frc.robot.commands.coroutines.*;
import frc.robot.commands.coroutines.extradriver.HighAlgaeGrabCoralFirstED;
import frc.robot.commands.coroutines.extradriver.HighAlgaeGrabED;
import frc.robot.commands.coroutines.extradriver.LowAlgaeGrabED;
import frc.robot.commands.coroutines.extradriver.ShootL2ED;
import frc.robot.commands.coroutines.extradriver.ShootL3ED;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.AlgaeClawConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.oi.CustomAPacOI;
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
    private final CustomAPacOI operatorBoard = new CustomAPacOI();

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
    private final Command leftCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReefUniversal(drivetrain, 0), driverController, 0.5);
    private final Command rightCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReefUniversal(drivetrain, 1), driverController, 0.5);
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
    private final Command grabAlgae = new GrabAlgae(algaeClaw);
    private final Command processAlgae = new ProcessAlgaeSubroutine(algaeClaw);

    // coroutines
    // write code for all coroutines under the coroutines folder here:
    private final Command restModeCoroutine = new RestMode(elevator, algaePivot, algaeClaw);
    private final Command lowAlgaeGrabCoroutine = new LowAlgaeGrab(drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command highAlgaeGrabCoroutine = new HighAlgaeGrab(drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command highAlgaeGrabCoralFirstCoroutine = new HighAlgaeGrabCoralFirstED(drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command shootL1Coroutine = new ShootL1(drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command shootL2LeftCoroutine = new ShootL2(false, drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command shootL2RightCoroutine = new ShootL2(true, drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command shootL3LeftCoroutine = new ShootL3(false, drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command shootL3RightCoroutine = new ShootL3(true, drivetrain, elevator, coralShooter, algaePivot, algaeClaw, driverController);
    private final Command intakeCoralCoroutine = new IntakeCoralFull(coralShooter);
    private final Command processAlgaeCoroutine = new ProcessAlgae(elevator, algaePivot, algaeClaw);

    public SendableChooser<Command> autoSelector;

    private void initiateNamedCoroutines() {
        NamedCommands.registerCommand("HighAlgaeCoroutine", highAlgaeGrabCoroutine);
        NamedCommands.registerCommand("LowAlgaeCoroutine", lowAlgaeGrabCoroutine);
        NamedCommands.registerCommand("ShootL1Coroutine", shootL1Coroutine);
        NamedCommands.registerCommand("ShootL2LeftCoroutine", shootL2LeftCoroutine);
        NamedCommands.registerCommand("ShootL2RightCoroutine", shootL2RightCoroutine);
        NamedCommands.registerCommand("ShootL3LeftCoroutine", shootL3LeftCoroutine);
        NamedCommands.registerCommand("ShootL3RightCoroutine", shootL3RightCoroutine);
        NamedCommands.registerCommand("ProcessAlgaeCoroutine", processAlgaeCoroutine);
        NamedCommands.registerCommand("RestModeCoroutine", restModeCoroutine);
        NamedCommands.registerCommand("IntakeCoralCoroutine", intakeCoralCoroutine);
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(swerveTeleop);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {

        operatorController.leftBumper().onTrue(intakeCoralCoroutine);
        operatorController.rightBumper().onTrue(new ScoreCoralL1(coralShooter));
        // operatorController.b().onTrue(scoreCoralL1);

        // algaeClaw.setDefaultCommand(algaeClawTeleop);
        operatorController.y().onTrue(grabAlgae);
        operatorController.a().onTrue(processAlgae);
        operatorController.x().onTrue(algaeClawOut);
        operatorController.b().onTrue(algaeRestMode);

        // elevator.setDefaultCommand(elevatorTeleop);
        // operatorController.y().onTrue(test1ElePreset);
        operatorController.a().onTrue(restMode);

        driverController.leftBumper().whileTrue(leftCoralAutoDrive);
        driverController.rightBumper().whileTrue(rightCoralAutoDrive);
        driverController.a().whileTrue(new CloseDriveToClosestReefGoodOffset(drivetrain));
        // driverController.x().whileTrue(reefAutoDrive);
        driverController.start().whileTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0)), drivetrain));
        // driverController.b().onTrue(processorAutoDrive);

        // Operator Board Shenanigans!
        operatorBoard.l3LeftButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL3LeftCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.l3RightButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL3RightCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.l2LeftButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL2LeftCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.l2RightButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL2RightCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.l1Button.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            scoreCoralL1
        ).onFalse(restModeCoroutine);
        operatorBoard.intakeButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            intakeCoralCoroutine
        );
        operatorBoard.algaeHighButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            highAlgaeGrabCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.algaeLowButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            lowAlgaeGrabCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.algaeProcessButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            processAlgaeCoroutine
        ).onFalse(restModeCoroutine);
        operatorBoard.restModeButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            restModeCoroutine
        );
    }

    private void initializeAutoCommands() {
        // auto commands
        final Command testL3Auto = new TestAlgaeL3Auto(drivetrain, elevator, coralShooter, algaeClaw, algaePivot);
        final Command twoL3AlgaeAuto = new TwoL3AlgaeAuto(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        final Command twoL3NoAlgaeAuto = new TwoL3NoAlgaeAuto(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        
        // autoselector
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("No Auto", new InstantCommand(() -> drivetrain.resetPose(drivetrain.getState().Pose), drivetrain));
        autoSelector.addOption("Test L3 + Grab Algae Auto", testL3Auto);
        autoSelector.addOption("Two L3 Algae Auto", twoL3AlgaeAuto);
        autoSelector.addOption("Two L3 NO Algae Auto", twoL3AlgaeAuto);
        autoSelector.close();
        SmartDashboard.putData("Auto Selector", autoSelector);
        
    }

    public RobotContainer() {
        initiateNamedCoroutines();
        configureDefaultCommands();
        configureBindings();
        initializeAutoCommands();
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
        // return null;
    }
}
