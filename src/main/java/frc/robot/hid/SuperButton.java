package frc.robot.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class SuperButton extends JoystickButton {
    private boolean currentState = false;
    private boolean previousState = false;

    public SuperButton(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
    }

    public boolean justClicked() {
        return !previousState && currentState;
    }

    public void onUpdate() {
        previousState = currentState;
        currentState = this.get();
    }
}