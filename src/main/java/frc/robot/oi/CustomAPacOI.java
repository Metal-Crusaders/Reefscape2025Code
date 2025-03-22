package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.coroutines.CoralShakeMode;
import frc.robot.constants.Constants;

public class CustomAPacOI {

    private Joystick aPac1, aPac2;
    public JoystickButton l3LeftButton, l3RightButton, l2LeftButton, l2RightButton, l1Button, intakeButton;
    public JoystickButton algaeHighButton, algaeLowButton, algaeProcessButton;
    public JoystickButton restModeButton, coralShakeModeButton;

    public CustomAPacOI() {
        aPac1 = new Joystick(Constants.OIConstants.APAC1);
        aPac2 = new Joystick(Constants.OIConstants.APAC2);

        configureButtons();
    }

    public void configureButtons() {
        l3LeftButton = new JoystickButton(aPac1, 1);
        l3RightButton = new JoystickButton(aPac1, 2);
        l2LeftButton = new JoystickButton(aPac1, 3);
        l2RightButton = new JoystickButton(aPac1, 4);
        l1Button = new JoystickButton(aPac1, 5);
        intakeButton = new JoystickButton(aPac1, 6);

        algaeHighButton = new JoystickButton(aPac1, 7);
        algaeLowButton = new JoystickButton(aPac1, 8);
        algaeProcessButton = new JoystickButton(aPac1, 11);

        restModeButton = new JoystickButton(aPac1, 12);

        coralShakeModeButton = new JoystickButton(aPac2, 1);
    }
    
    
}
