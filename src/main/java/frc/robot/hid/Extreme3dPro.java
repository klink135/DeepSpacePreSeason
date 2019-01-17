package frc.robot.hid;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Extreme3dPro extends SuperJoystick {
    public JoystickButton trigger = new JoystickButton(this, 1);
    public JoystickButton thumbButton = new JoystickButton(this, 2);
    public JoystickButton leftLower = new JoystickButton(this, 3);
    public JoystickButton rightLower = new JoystickButton(this, 4);
    public JoystickButton leftUpper = new JoystickButton(this, 5);
    public JoystickButton rightUpper = new JoystickButton(this, 6);
    public JoystickButton button7 = new JoystickButton(this, 7);
    public JoystickButton button8 = new JoystickButton(this, 8);
    public JoystickButton button9 = new JoystickButton(this, 9);
    public JoystickButton button10 = new JoystickButton(this, 10);
    public JoystickButton button11 = new JoystickButton(this, 11);
    public JoystickButton button12 = new JoystickButton(this, 12);

    // TODO middle dpad, bottom slider, x, y, twist

    public Extreme3dPro(int port) {
        super(port);
    }

    @Override
    public void onUpdate() {

    }

    @Override
    public void outputTelemetry() {

    }
}