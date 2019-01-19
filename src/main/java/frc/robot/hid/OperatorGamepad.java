package frc.robot.hid;

import edu.wpi.first.wpilibj.XboxController;

// FIXME
public class OperatorGamepad {
    private static OperatorGamepad instance;
    private XboxController operatorJoystick;

    public static OperatorGamepad getInstance() {
        if (instance == null) {
            instance = new OperatorGamepad();
        }
        return instance;
    }

    private OperatorGamepad() {
        operatorJoystick = new XboxController(1);
    }

    public boolean liftToMidHatch() {
        return operatorJoystick.bButton.get();
    }

    public boolean liftToLowHatch() {
        return operatorJoystick.aButton.get();
    }

    public boolean liftToHighHatch() {
        return operatorJoystick.yButton.get();
    }

    public boolean disengageButtonTapped() {
        return operatorJoystick.xButton.justClicked();
    }

    public double getGripperPower() {
        return operatorJoystick.getLeftStickY();
    }
}