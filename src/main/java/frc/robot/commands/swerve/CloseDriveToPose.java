package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPose extends Command {

    private final CommandSwerveDrivetrain swerve;
    private Pose2d poseFinal, currentPose;

    private final PIDController xTranslationPID, yTranslationPID;
    private final PIDController rotationPID;

    public CloseDriveToPose(CommandSwerveDrivetrain swerve, Pose2d finalPose) {
        this.swerve = swerve;
        this.poseFinal = finalPose;
        this.xTranslationPID = new PIDController(Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KP, 
                                                Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KI, 
                                                Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KD);
        this.yTranslationPID = new PIDController(Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KP, 
                                                Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KI, 
                                                Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KD);
        this.rotationPID = new PIDController(Constants.SwerveConstants.CLOSE_ROTATION_PP_KP, 
                                             Constants.SwerveConstants.CLOSE_ROTATION_PP_KI, 
                                             Constants.SwerveConstants.CLOSE_ROTATION_PP_KD);

        this.rotationPID.enableContinuousInput(-1 * Math.PI, Math.PI);
        
        xTranslationPID.setTolerance(0.005);
        yTranslationPID.setTolerance(0.005);
        rotationPID.setTolerance(0.1);
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentPose = swerve.getState().Pose;

        xTranslationPID.reset();
        yTranslationPID.reset();
        rotationPID.reset();

        xTranslationPID.setSetpoint(poseFinal.getX());
        yTranslationPID.setSetpoint(poseFinal.getY());
        rotationPID.setSetpoint(poseFinal.getRotation().getRadians());
    }

    @Override
    public void execute() {
        currentPose = swerve.getState().Pose;
        
        double xSpeed = xTranslationPID.calculate(currentPose.getX());
        double ySpeed = yTranslationPID.calculate(currentPose.getY());
        double thetaSpeed = rotationPID.calculate(currentPose.getRotation().getRadians());

        // SmartDashboard.putNumber("Current PID Setpoint", 0)
        
        ChassisSpeeds wheelSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        swerve.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(wheelSpeeds));
    }

    @Override
    public boolean isFinished() {
        boolean xTranslationDone = xTranslationPID.atSetpoint();
        boolean yTranslationDone = yTranslationPID.atSetpoint();
        boolean rotationDone = rotationPID.atSetpoint();
        
        return xTranslationDone && yTranslationDone && rotationDone;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
