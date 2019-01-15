package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.HAL;

public class XboxController extends Joystick {

    public XboxController(int port) {
        super(port);
    }

    public SuperButton xButton = new SuperButton(this, 3);
    public SuperButton yButton = new SuperButton(this, 4);
    public SuperButton aButton = new SuperButton(this, 1);
    public SuperButton bButton = new SuperButton(this, 2);
    public SuperButton rightBumper = new SuperButton(this, 6);
    public SuperButton leftBumper = new SuperButton(this, 5);
    public SuperButton startButton = new SuperButton(this, 8);
    public SuperButton selectButton = new SuperButton(this, 7);
    public SuperButton leftStickButton = new SuperButton(this, 9);
    public SuperButton rightStickButton = new SuperButton(this, 10);
    public SuperButton leftTriggerButton = new TriggerBoolean(this, 2);
    public SuperButton rightTriggerButton = new TriggerBoolean(this, 3);

    public void onUpdate() {
        xButton.onUpdate();
        yButton.onUpdate();
        aButton.onUpdate();
        bButton.onUpdate();
        rightBumper.onUpdate();
        leftBumper.onUpdate();
        startButton.onUpdate();
        selectButton.onUpdate();
        leftStickButton.onUpdate();
        rightStickButton.onUpdate();
        leftTriggerButton.onUpdate();
        rightTriggerButton.onUpdate();
    }

    public void outputTelemetry() {

    }

    private int m_outputs;
    private short m_leftRumble;
    private short m_rightRumble;

    // public DPadUp dPadUp = new DPadUp(this);
    // public DPadDown dPadDown = new DPadDown(this);

    public double getTriggerTwist() {
        double leftTriggerValue = this.getRawAxis(2);
        double rightTriggerValue = -1 * this.getRawAxis(3);
        return leftTriggerValue + rightTriggerValue;
    }

    public double getLeftStickX() {
        return this.getRawAxis(0);
    }

    public double getLeftStickY() {
        return -this.getRawAxis(1);
    }

    public double getRightStickX() {
        return this.getRawAxis(4); // 4
    }

    public double getRightStickY() {
        return -this.getRawAxis(5);
    }

    public void setRumble(double leftValue, double rightValue) {
        setRumble(RumbleType.kLeftRumble, leftValue);
        setRumble(RumbleType.kRightRumble, rightValue);
    }

    public void setRumble(RumbleType type, double value) {
        if (value < 0) {
            value = 0;
        } else if (value > 1) {
            value = 1;
        }
        if (type == RumbleType.kLeftRumble) {
            m_leftRumble = (short) (value * 65535);
        } else {
            m_rightRumble = (short) (value * 65535);
        }
        HAL.setJoystickOutputs((byte) getPort(), m_outputs, m_leftRumble, m_rightRumble);
    }
}