// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.commands.swerve.DriveToReef;
import frc.robot.commands.swerve.DriverAffectedDriveToPose;
import frc.robot.commands.swerve.SmartDriveToReef;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.swerve.AutoDriveProcessor;

import frc.robot.commands.swerve.AutoLineUpReefUniversal;
import frc.robot.commands.swerve.CloseDriveToClosestReefGoodOffset;
import frc.robot.commands.swerve.CloseDriveToPose;
import frc.robot.commands.swerve.SwerveTeleopShortTerm;
import frc.robot.commands.utils.CoralConditionalCommand;
import frc.robot.commands.utils.DisplayImageCommand;
import frc.robot.commands.utils.DriverInterruptible;
import frc.robot.commands.utils.JoystickInterruptible;
import frc.robot.commands.auto.center.CenterL3AndDoubleProcessAuto;
import frc.robot.commands.auto.center.TestAlgaeL3Auto;
import frc.robot.commands.auto.openSide.L2AndTwoL3AutoOpenSide;
import frc.robot.commands.auto.openSide.TwoL3AndL2AutoOpenSide;
import frc.robot.commands.auto.openSide.TwoL3BuddyAutoOpenSide;
import frc.robot.commands.auto.processorSide.BumpTwoL3AlgaeAutoProcessorSide;
import frc.robot.commands.auto.processorSide.L2AndTwoL3AutoProcessorSide;
import frc.robot.commands.auto.processorSide.TwoL3AlgaeAutoProcessorSide;
import frc.robot.commands.auto.processorSide.TwoL3AndL2AutoProcessorSide;
import frc.robot.commands.coroutines.*;
import frc.robot.commands.coroutines.extradriver.HighAlgaeGrabCoralFirstED;
import frc.robot.commands.coroutines.extradriver.HighAlgaeGrabED;
import frc.robot.commands.coroutines.extradriver.LowAlgaeGrabED;
import frc.robot.commands.coroutines.extradriver.ShootL2ED;
import frc.robot.commands.coroutines.extradriver.ShootL3ED;
import frc.robot.commands.coroutines.nodriver.HighAlgaeGrabCoralFirstNoDriver;
import frc.robot.commands.coroutines.nodriver.LowAlgaeGrabNoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL2NoDriver;
import frc.robot.commands.coroutines.nodriver.ShootL3NoDriver;
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
import frc.robot.subsystems.util.ReefAlignmentPID;
import frc.robot.subsystems.util.ReefDisplay;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    // OI STUFF
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CustomAPacOI operatorBoard = new CustomAPacOI();

    // SUBSYSTEMS
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    public final ReefAlignmentPID reefAlignmentPID = new ReefAlignmentPID(drivetrain);
    public final ReefDisplay reefDisplay = new ReefDisplay();

    public final Elevator elevator = new Elevator();
    public final CoralShooter coralShooter = new CoralShooter();
    public final AlgaeClaw algaeClaw = new AlgaeClaw();
    public final AlgaePivot algaePivot = new AlgaePivot();

    // CAMERA STUFF // TODO UNCOMMENT THIS WHEN PHOTONVISION STUFF COMES OUT
    public AprilTagCamera camera1 = new AprilTagCamera(Constants.CameraConstants.CAMERA_1_NAME, Constants.CameraConstants.CAMERA_1_POS, drivetrain);
    public AprilTagCamera camera2 = new AprilTagCamera(Constants.CameraConstants.CAMERA_2_NAME, Constants.CameraConstants.CAMERA_2_POS, drivetrain);
    // public AprilTagCamera camera3 = new AprilTagCamera(Constants.CameraConstants.CAMERA_3_NAME, Constants.CameraConstants.CAMERA_3_POS, drivetrain);

    // COMMANDS!!

    // drivetrain

    private final Command swerveTeleop = new SwerveTeleop(drivetrain, driverController);
    private final Command leftCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReefUniversal(drivetrain, 0), driverController, 0.5);
    private final Command rightCoralAutoDrive = new JoystickInterruptible(new AutoLineUpReefUniversal(drivetrain, 1), driverController, 0.5);
    private final Command processorAutoDrive = new JoystickInterruptible(new AutoDriveProcessor(drivetrain), driverController, 0.5);
    // private final Command reefAutoDrive = new JoystickInterruptible(new DriveToReef(drivetrain), driverController, 0.5);

    // elevator commands
    // private final Command elevatorTeleop = new ElevatorTeleop(elevator, operatorController);
    private final ElevatorPreset restMode = new ElevatorPreset(elevator, Constants.ElevatorConstants.L1_ENCODER_TICKS);
    private final ElevatorPreset test1ElePreset = new ElevatorPreset(elevator, Constants.ElevatorConstants.L2_ENCODER_TICKS);

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

    // reef coroutines
    private final Command closeCenterAutoDrive = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 1), driverController, 0.5);
    private final Command closeLeftAutoDrive   = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 0), driverController, 0.5);
    private final Command closeRightAutoDrive  = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 2), driverController, 0.5);
    private final Command farCenterAutoDrive   = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 4), driverController, 0.5);
    private final Command farLeftAutoDrive     = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 5), driverController, 0.5);
    private final Command farRightAutoDrive    = new JoystickInterruptible(new DriveToReef(drivetrain, reefDisplay, reefAlignmentPID, 3), driverController, 0.5);

    // coroutines
    // write code for all coroutines under the coroutines folder here:
    private final Command restModeCoroutine = new RestMode(elevator, algaePivot, algaeClaw);
    private final Command coralShakeCoroutine = new CoralShakeMode(elevator, algaePivot, algaeClaw);
    private final Command lowAlgaeGrabCoroutine = new LowAlgaeGrabNoDriver(drivetrain, elevator, coralShooter, algaePivot, algaeClaw);
    private final Command highAlgaeGrabCoralFirstCoroutine = new HighAlgaeGrabCoralFirstNoDriver(drivetrain, elevator, coralShooter, algaePivot, algaeClaw);
    private final Command shootL2LeftCoroutine = new CoralConditionalCommand(new ShootL2NoDriver(false, drivetrain, elevator, coralShooter, algaePivot, algaeClaw), coralShooter);
    private final Command shootL2RightCoroutine = new CoralConditionalCommand(new ShootL2NoDriver(true, drivetrain, elevator, coralShooter, algaePivot, algaeClaw), coralShooter);
    private final Command shootL3LeftCoroutine = new CoralConditionalCommand(new ShootL3NoDriver(false, drivetrain, elevator, coralShooter, algaePivot, algaeClaw), coralShooter);
    private final Command shootL3RightCoroutine = new CoralConditionalCommand(new ShootL3NoDriver(true, drivetrain, elevator, coralShooter, algaePivot, algaeClaw), coralShooter);
    private final Command intakeCoralCoroutine = new IntakeCoralFull(coralShooter);
    private final Command processAlgaeCoroutine = new ProcessAlgae(elevator, algaePivot, algaeClaw);

    public SendableChooser<Command> autoSelector;
    public SendableChooser<Command> reefAutoDriveSelector = new SendableChooser<>();

    private void initiateNamedCoroutines() {
        NamedCommands.registerCommand("ProcessAlgaeSubroutine", processAlgaeCoroutine);
        NamedCommands.registerCommand("RestModeCoroutine", restModeCoroutine);
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
        // operatorController.y().onTrue(grabAlgae);
        // operatorController.a().onTrue(processAlgae);
        // operatorController.x().onTrue(algaeClawOut);
        // operatorController.b().onTrue(algaeRestMode);

        // elevator.setDefaultCommand(elevatorTeleop);
        operatorController.y().onTrue(test1ElePreset);
        operatorController.a().onTrue(restMode);

        // driverController.leftBumper().whileTrue(leftCoralAutoDrive);
        // driverController.rightBumper().whileTrue(rightCoralAutoDrive);
        // driverController.a().whileTrue(new CloseDriveToClosestReefGoodOffset(drivetrain));
        // driverController.x().whileTrue(reefAutoDrive);
        driverController.start().whileTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0)), drivetrain));
        driverController.back().whileTrue(processorAutoDrive);

        // driverController.leftBumper().onTrue(
        //     new SequentialCommandGroup(
        //         new CloseDriveToClosestReefGoodOffset(drivetrain),
        //         new AutoLineUpReefUniversal(drivetrain, 0)
        //     )
        // );
        // driverController.rightBumper().onTrue(
        //     new SequentialCommandGroup(
        //         new CloseDriveToClosestReefGoodOffset(drivetrain),
        //         new AutoLineUpReefUniversal(drivetrain, 1)
        //     )
        // );

        // SysID Routines
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Operator Board Shenanigans!
        operatorBoard.l3LeftButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL3LeftCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.l3RightButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL3RightCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.l2LeftButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL2LeftCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.l2RightButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            shootL2RightCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.l1Button.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            scoreCoralL1
        );
        operatorBoard.intakeButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            intakeCoralCoroutine
        );
        operatorBoard.algaeHighButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            highAlgaeGrabCoralFirstCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.algaeLowButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            lowAlgaeGrabCoroutine
        ); // .onFalse(restModeCoroutine);
        operatorBoard.algaeProcessButton.debounce(Constants.OIConstants.SCORE_DEBOUNCING_TIME).onTrue(
            processAlgaeCoroutine
        );
        operatorBoard.restModeButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            restModeCoroutine
        );
        operatorBoard.coralShakeModeButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            coralShakeCoroutine
        );

        // Reef Buttons
        operatorBoard.closeCenterButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            closeCenterAutoDrive
        );
        operatorBoard.closeLeftButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            closeLeftAutoDrive
        );
        operatorBoard.closeRightButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            closeRightAutoDrive
        );
        operatorBoard.farCenterButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            farCenterAutoDrive
        );
        operatorBoard.farLeftButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            farLeftAutoDrive
        );
        operatorBoard.farRightButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            farRightAutoDrive
        );
        operatorBoard.reefDriveRestButton.debounce(Constants.OIConstants.REST_DEBOUNCING_TIME).onTrue(
            new SequentialCommandGroup(
                new DisplayImageCommand(reefDisplay, 6),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(closeCenterAutoDrive)),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(closeLeftAutoDrive)),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(closeRightAutoDrive)),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(farCenterAutoDrive)),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(farLeftAutoDrive)),
                new InstantCommand(() -> CommandScheduler.getInstance().cancel(farRightAutoDrive))
            )
        );
    }

    private void initializeAutoCommands() {
        // auto commands
        final Command centerL3ProcessAlgaeAuto = new CenterL3AndDoubleProcessAuto(drivetrain, elevator, coralShooter, algaeClaw, algaePivot);
        final Command twoL3AlgaeAutoProcessorSide = new TwoL3AlgaeAutoProcessorSide(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        final Command twoL3AndL2ProcessorSide = new L2AndTwoL3AutoProcessorSide(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        final Command twoL3ProcessAlgaeBumpProcessorSide = new BumpTwoL3AlgaeAutoProcessorSide(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        final Command twoL3BumpOpenSide = new TwoL3BuddyAutoOpenSide(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);
        final Command twoL3AndL2OpenSide = new L2AndTwoL3AutoOpenSide(drivetrain, elevator, algaeClaw, algaePivot, coralShooter);

        // autoselector
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("No Auto", new InstantCommand(() -> drivetrain.resetPose(drivetrain.getState().Pose), drivetrain));
        autoSelector.addOption("L3 + Double Process Algae Auto - Center", centerL3ProcessAlgaeAuto);
        autoSelector.addOption("Two L3 Process Algae Auto - Processor Side", twoL3AlgaeAutoProcessorSide);
        autoSelector.addOption("Two L3 + L2 Auto - Processor Side", twoL3AndL2ProcessorSide);
        autoSelector.addOption("Two L3 + Process Algae + Buddy Taxi - Processor Side", twoL3ProcessAlgaeBumpProcessorSide);
        autoSelector.addOption("Two L3 + L2 - Open Side", twoL3AndL2OpenSide);
        autoSelector.addOption("Two L3 + Buddy Taxi - Open Side", twoL3BumpOpenSide);
        
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
        // return new SequentialCommandGroup(
        //     new CloseDriveToClosestReefGoodOffset(drivetrain),
        //     new AutoLineUpReefUniversal(drivetrain, 1)
        // );
    }

}
