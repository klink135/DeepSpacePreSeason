package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorGamepad {
    private static OperatorGamepad instance;
    private Joystick operatorJoystick;

    public static OperatorGamepad getInstance(){
        if(instance == null){
            instance = new OperatorGamepad();
        }
        return instance;
    }

    private OperatorGamepad(){
        operatorJoystick = new Joystick(1);
    }

    public boolean liftToMidHatch(){
        return operatorJoystick.getRawButton(2);
    }

    public boolean liftToLowHatch(){
        return operatorJoystick.getRawButton(1);
    }

    public boolean liftToHighHatch(){
        return operatorJoystick.getRawButton(4);
    }

    public boolean disengageButton(){
        return operatorJoystick.getRawButton(3);
    }

    public double getGripperPower(){
        return operatorJoystick.getRawAxis(1);
    }
}