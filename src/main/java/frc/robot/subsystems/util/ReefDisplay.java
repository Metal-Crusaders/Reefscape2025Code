package frc.robot.subsystems.util;

import java.io.File;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefDisplay extends SubsystemBase {

    private CvSource imageStream;


    public ReefDisplay() {
        CameraServer.addServer("autodrive", 1185);
            
        imageStream = CameraServer.putVideo("Auto Drive Position", 640, 480);
        imageStream.setConnectionStrategy(CvSource.ConnectionStrategy.kKeepOpen);
        imageStream.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        // Start a new CameraServer stream if not already running

        CameraServer.getServer("autodrive").setSource(imageStream);
    }
    
    public void setReef(int imageNumber) {
        if (imageNumber < 0 || imageNumber > 6) {
            System.out.println("Invalid image number: " + imageNumber);
            return;
        }

        // Load the image
        String imagePath = "/home/lvuser/deploy/reefPoses/" + imageNumber + ".png";
        File imageFile = new File(imagePath);
        if (!imageFile.exists()) {
            System.out.println("Image file not found: " + imagePath);
            return;
        }

        Mat image = Imgcodecs.imread(imagePath); // Read the image into OpenCV Mat
        if (image.empty()) {
            System.out.println("Failed to load image: " + imagePath);
            return;
        }

        // Send the image as a video frame
        imageStream.putFrame(image);
    }


}
