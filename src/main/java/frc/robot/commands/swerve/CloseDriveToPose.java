package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPose extends Command {

    private final CommandSwerveDrivetrain swerve;
    private Pose2d poseFinal, currentPose, prevPose;

    private final PIDController xTranslationPID, yTranslationPID;
    private final PIDController rotationPID;

    private final double POSE_TRANSLATION_UPDATE_TOLERANCE = 0.01;
    private final double POSE_ROTATION_UPDATE_TOLERANCE = 0.01; // TODO ADJUST
    private final double CHECKING_PERIOD = 0.2;
    private final double COOL_DOWN = 0.35;

    private boolean cooldown;

    private final Timer timer;

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
        
        xTranslationPID.setTolerance(0.1);
        yTranslationPID.setTolerance(0.1);
        rotationPID.setTolerance(0.01);

        timer = new Timer();
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        cooldown = false;
        
        currentPose = swerve.getState().Pose;

        prevPose = new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d());

        xTranslationPID.reset();
        yTranslationPID.reset();
        rotationPID.reset();

        xTranslationPID.setSetpoint(poseFinal.getX());
        yTranslationPID.setSetpoint(poseFinal.getY());
        rotationPID.setSetpoint(poseFinal.getRotation().getRadians());

        timer.reset();
        timer.start();
        SmartDashboard.putBoolean("Updated Checking Period", false);
    }

    @Override
    public void execute() {
        currentPose = swerve.getState().Pose;
        
        double xSpeed = xTranslationPID.calculate(currentPose.getX());
        double ySpeed = yTranslationPID.calculate(currentPose.getY());
        double thetaSpeed = rotationPID.calculate(currentPose.getRotation().getRadians());
        
        ChassisSpeeds wheelSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        swerve.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(wheelSpeeds));

        if (timer.hasElapsed(CHECKING_PERIOD) && cooldown) {
            timer.reset();
            timer.start();
            prevPose = currentPose;
            SmartDashboard.putBoolean("Updated Checking Period", true);
        }
        
        if (timer.hasElapsed(COOL_DOWN)) {
            timer.start();
            timer.reset();
            cooldown = true;
        } 

        SmartDashboard.putNumber("X Updated", Math.abs(currentPose.getX() - prevPose.getX()));
        SmartDashboard.putNumber("Y Updated", Math.abs(currentPose.getY() - prevPose.getY()));
        
    }

    @Override
    public boolean isFinished() {
        boolean xTranslationDone = xTranslationPID.atSetpoint();
        boolean yTranslationDone = yTranslationPID.atSetpoint();
        boolean rotationDone = rotationPID.atSetpoint();

        boolean xTranslationUpdated = Math.abs(currentPose.getX() - prevPose.getX()) > POSE_TRANSLATION_UPDATE_TOLERANCE;
        boolean yTranslationUpdated = Math.abs(currentPose.getY() - prevPose.getY()) > POSE_TRANSLATION_UPDATE_TOLERANCE;
        boolean rotationUpdated = Math.abs(currentPose.getRotation().getRadians() - prevPose.getRotation().getRadians()) > POSE_ROTATION_UPDATE_TOLERANCE;

        return (xTranslationDone && yTranslationDone && rotationDone) || 
               (!(xTranslationUpdated) && !(yTranslationUpdated) && !(rotationUpdated));
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
        timer.stop();
        cooldown = false;
    }
}
