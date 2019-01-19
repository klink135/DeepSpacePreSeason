/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hid.OperatorGamepad;
import frc.robot.subsystem.PathCapableDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class PathRobot2 extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "Path Follpwing Test";
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private PathCapableDrive drive = PathCapableDrive.getInstance();
    public final Joystick driverJoystick = new Joystick(0);
    private final OperatorGamepad operatorGamepad = OperatorGamepad.getInstance();
    private final Spark gripperWrist = new Spark(2);

    @Override
    public void robotInit() {
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Path Following Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Gamepad", leftStickY);
        SmartDashboard.putNumber("Right Gamepad", rightStickY);
        drive.outputTelemetry();
    }

    @Override
    public void autonomousInit() {
        drive.startPath();
    }

    @Override
    public void autonomousPeriodic() {
        drive.onLoop(Timer.getFPGATimestamp());
    }

    double leftStickY = 0.0;
    double rightStickY = 0.0;
    
    @Override
    public void teleopPeriodic() {
        leftStickY = operatorGamepad.getLeftStickY();
        rightStickY = operatorGamepad.getRightStickY();
        drive.rawTankDrive(leftStickY, rightStickY);
        drive.onLoop(Timer.getFPGATimestamp());
        // drive.arcadeDrive(-driverJoystick.getRawAxis(1), driverJoystick.getRawAxis(0));
        // double multiplier = operatorGamepad.liftToHighHatch() ? 0.8 : 0.4;
        // gripperWrist.set(operatorGamepad.getGripperPower() * multiplier);
    }

    @Override
    public void testPeriodic() {
    }
}
