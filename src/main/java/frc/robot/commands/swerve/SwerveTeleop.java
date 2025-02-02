package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private static final double MAX_TRANSLATION_SPEED = 3.0; // meters per second
    private static final double MAX_ROTATION_SPEED = Math.PI; // radians per second
    private static final double DEADBAND = 0.15;

    private double targetAngle, prevTargetAngle = 0;

    // PID controller for rotational interpolation
    private final PIDController rotationPID;

    // Define target snap angles in radians
    private static final double[] SNAP_ANGLES = {
        0.0, 3 * Math.PI / 2, Math.PI, Math.PI / 2
    };

    public SwerveTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        
        // Initialize the PID controller for rotation
        // TODO CUSTOMIZE THIS!
        rotationPID = new PIDController(5.0, 0.0, 0.2); // Tuned PID gains
        rotationPID.setTolerance(0.1); // Tolerance for stopping rotation
        rotationPID.setIntegratorRange(-MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

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

        // Determine if snapping to an angle is requested
        targetAngle = getSnapTargetAngle(rotation);
        // if (targetAngle == 1e9) {
        //     targetAngle = prevTargetAngle;
        // }
        // if (targetAngle != -1) {
        //     // Use PID to interpolate rotational rate
        //     double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();

        //     // Calculate the shortest path to the target angle
        //     double angleDifference = targetAngle - currentAngle;
        //     angleDifference = Math.IEEEremainder(angleDifference, 2 * Math.PI); // Normalize to [-pi, pi]

        //     // Update PID controller with the shortest path
        //     rotation = rotationPID.calculate(currentAngle, currentAngle + angleDifference);

        //     // Clamp the rotation rate to the max allowable speed
        //     rotation = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotation));
        // }

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

        prevTargetAngle = targetAngle;
        
    }

    private double getSnapTargetAngle(double rotation) {
        if (rotation != 0) {
            return -1;
        }
        if (controller.y().getAsBoolean()) {
            return SNAP_ANGLES[0]; // Forward
        } else if (controller.b().getAsBoolean()) {
            return SNAP_ANGLES[1]; // Right
        } else if (controller.a().getAsBoolean()) {
            return SNAP_ANGLES[2]; // Backward
        } else if (controller.x().getAsBoolean()) {
            return SNAP_ANGLES[3]; // Left
        }
        return 1e9; // No snap requested
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            prevTargetAngle = -1;
        }

        // Stop the drivetrain
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0.0;
    }
}
