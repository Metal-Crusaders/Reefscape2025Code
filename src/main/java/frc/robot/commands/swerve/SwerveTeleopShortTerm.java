package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveTeleopShortTerm extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private static final double MAX_TRANSLATION_SPEED = 0.75; // meters per second
    private static final double MAX_ROTATION_SPEED = Math.PI * 0.25; // radians per second
    private static final double DEADBAND = 0.15;

    private double targetAngle, prevTargetAngle = 0;

    // PID controller for rotational interpolation
    private final PIDController rotationPID;

    public SwerveTeleopShortTerm(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        
        // Initialize the PID controller for rotation
        // TODO CUSTOMIZE THIS!
        rotationPID = new PIDController(2.0, 0.0, 0.2); // Tuned PID gains
        rotationPID.setTolerance(0.01); // Tolerance for stopping rotation
        rotationPID.setIntegratorRange(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SwerveRequest.FieldCentricFacingAngle fieldCentricRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(drivetrain.getState().RawHeading)
            .withCenterOfRotation(new Translation2d(0, 0));
        drivetrain.setControl(fieldCentricRequest);
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double translationX = -controller.getLeftY();
        double translationY = -controller.getLeftX();
        double rotation = -controller.getRightX();

        // Apply deadband
        translationX = applyDeadband(translationX);
        translationY = applyDeadband(translationY);
        rotation = applyDeadband(rotation);

        // Scale inputs to max speeds
        translationX *= MAX_TRANSLATION_SPEED;
        translationY *= MAX_TRANSLATION_SPEED;
        rotation *= MAX_ROTATION_SPEED;

        // Create a field-centric swerve request
        SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withVelocityX(translationX)
            .withVelocityY(translationY)
            .withRotationalRate(rotation)
            .withCenterOfRotation(new Translation2d(0, 0));
        drivetrain.setControl(fieldCentricRequest);

        // Update SmartDashboard for debugging
        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Rotational Output", rotation);
        SmartDashboard.putBoolean("Start Btn Slow Cmd", controller.x().getAsBoolean());

        prevTargetAngle = targetAngle;
        
    }
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return controller.x().getAsBoolean();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0.0;
    }
}
