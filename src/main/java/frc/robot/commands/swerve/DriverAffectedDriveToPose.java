package frc.robot.commands.swerve;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// NEW PLAN: create the path, drive along the path, when user interrupts, factor that in at a scalar percentage, and then implement a PID controller to
// correct back to the path over time

public class DriverAffectedDriveToPose extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final CommandXboxController driverController;
    private Pose2d poseFinal, currentPose, prevPose;

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private List<PathPlannerTrajectoryState> trajectoryStates;
    private Pose2d[] pathPoses;

    private final PIDController xTranslationPID, yTranslationPID;
    private final PIDController rotationPID;

    private final double POSE_TRANSLATION_UPDATE_TOLERANCE = 0.01;
    private final double POSE_ROTATION_UPDATE_TOLERANCE = 0.01; // TODO ADJUST
    private final double CLOSE_DRIVE_THRESHOLD = 0.25;
    private final double CHECKING_PERIOD = 0.15;
    private final double COOL_DOWN = 0.35;

    private boolean cooldown;

    private final Timer timer;

    public DriverAffectedDriveToPose(CommandSwerveDrivetrain swerve, Pose2d finalPose, CommandXboxController driverController) {
        this.swerve = swerve;
        this.poseFinal = finalPose;
        this.driverController = driverController;
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

        xTranslationPID.setTolerance(0.025);
        yTranslationPID.setTolerance(0.025);
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

        path = swerve.pathfind(poseFinal);
        
        while (path == null) {
            path = swerve.pathfinder.getCurrentPath(swerve.constraints, new GoalEndState(0, poseFinal.getRotation()));
        }

        try {
            trajectory = path.generateTrajectory(new ChassisSpeeds(0, 0, 0), swerve.getState().RawHeading, RobotConfig.fromGUISettings());
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
        trajectoryStates = trajectory.getStates();
        pathPoses = new Pose2d[trajectoryStates.size()];
        for (int i = 0; i < trajectoryStates.size(); i++) {
            pathPoses[i] = trajectoryStates.get(i).pose;
        }

        timer.reset();
        timer.start();
        SmartDashboard.putBoolean("Updated Checking Period", false);
    }

    @Override
    public void execute() {
        currentPose = swerve.getState().Pose;

        double xSpeed, ySpeed, thetaSpeed;
        ChassisSpeeds wheelSpeeds;


        if (currentPose.getTranslation().getDistance(poseFinal.getTranslation()) > CLOSE_DRIVE_THRESHOLD) {

            PathPlannerTrajectoryState bestTrajState = trajectoryStates.get(MathUtils.findClosestIdx(currentPose, pathPoses));
            wheelSpeeds = bestTrajState.fieldSpeeds;

        } else {
            xSpeed = xTranslationPID.calculate(currentPose.getX());
            ySpeed = yTranslationPID.calculate(currentPose.getY());
            thetaSpeed = rotationPID.calculate(currentPose.getRotation().getRadians());

            wheelSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }
        
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
