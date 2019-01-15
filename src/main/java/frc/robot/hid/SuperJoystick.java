package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

public abstract class SuperJoystick extends Joystick {
    public SuperJoystick(int port) {
        super(port);
    }

    public abstract void onUpdate();

    public abstract void outputTelemetry();
}