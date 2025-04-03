package frc.robot.commands.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.util.ReefDisplay;

import org.opencv.core.Mat;
import org.opencv.dnn.Image2BlobParams;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;

public class DisplayImageCommand extends Command {

    ReefDisplay reefDisplay;

    private final int imageNumber;

    public DisplayImageCommand(ReefDisplay reefDisplay, int imageNumber) {
        this.reefDisplay = reefDisplay;
        this.imageNumber = imageNumber;
    }


    @Override
    public void initialize() {

        reefDisplay.setReef(imageNumber);
        
    }

    @Override
    public boolean isFinished() {
        return true; // Runs once and exits
    }
}
