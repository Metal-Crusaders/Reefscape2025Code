package frc.robot.subsystems.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.DriveToReef;

public class ReefCommandManager {

    private Command reefDriveCmd = null;

    private final Command closeCenterAutoDrive;
    private final Command closeLeftAutoDrive  ;
    private final Command closeRightAutoDrive ;
    private final Command farCenterAutoDrive  ;
    private final Command farLeftAutoDrive    ;
    private final Command farRightAutoDrive   ;

    private Command[] reefCmds;

    public ReefCommandManager(Command closeCenterAutoDrive, Command closeLeftAutoDrive, Command closeRightAutoDrive,
                            Command farCenterAutoDrive, Command farLeftAutoDrive, Command farRightAutoDrive) {
        this.closeCenterAutoDrive = closeCenterAutoDrive;
        this.closeLeftAutoDrive   = closeLeftAutoDrive  ;
        this.closeRightAutoDrive  = closeRightAutoDrive ;
        this.farCenterAutoDrive   = farCenterAutoDrive  ;
        this.farLeftAutoDrive     = farLeftAutoDrive    ;
        this.farRightAutoDrive    = farRightAutoDrive   ;

        reefCmds = new Command[]{this.closeLeftAutoDrive, this.closeCenterAutoDrive, this.closeRightAutoDrive,
            this.farRightAutoDrive, this.farCenterAutoDrive, this.farLeftAutoDrive};
    }

    public Command getReefDriveCmd() {
        return reefDriveCmd;
    }

    public void setReefDriveCmd(int reefPose) {
        reefDriveCmd = reefCmds[reefPose];
    }

    public void clearReefDriveCmd() {
        reefDriveCmd = new PrintCommand("Cleared reef drive command set");
    }
    
}
