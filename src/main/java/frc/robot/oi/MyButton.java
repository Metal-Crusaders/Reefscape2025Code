package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class MyButton extends JoystickButton {
    private GenericHID hid;
    private int buttonNumber;

    public MyButton(GenericHID hid, int buttonNumber) {
        super(hid, buttonNumber);
        this.hid = hid;
        this.buttonNumber = buttonNumber;
    }

    public boolean getRaw() {
        return hid.getRawButton(buttonNumber);
    }

    public boolean isPressed() {
        return hid.getRawButtonPressed(buttonNumber);
    }

    public boolean isReleased() {
        return hid.getRawButtonReleased(buttonNumber);
    }

}