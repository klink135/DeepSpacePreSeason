/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.PathCapableDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class PathRobot2 extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "Path Following Test";
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private PathCapableDrive drive = PathCapableDrive.getInstance();
    public final Joystick driverJoystick = new Joystick(0);

    @Override
    public void robotInit() {
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Path Following Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
    }

    @Override
    public void robotPeriodic() {
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
    @Override
    public void teleopPeriodic() {
        drive.arcadeDrive(-driverJoystick.getRawAxis(1), driverJoystick.getRawAxis(0));
        drive.onLoop(Timer.getFPGATimestamp());
    }

    @Override
    public void testPeriodic() {
    }
}
