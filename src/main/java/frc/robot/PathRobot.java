/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.hid.OperatorGamepad;
import frc.robot.subsystem.Drive;
import frc.robot.subsystem.Lift;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXL345_SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import java.util.HashMap;

import org.opencv.core.Range;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class PathRobot extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "Path Follpwing Test";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();

    EncoderFollower leftFollow;
    EncoderFollower rightFollow;
    Trajectory.Config config;
    Waypoint[] points;
    Trajectory trajectory;
    TankModifier modifier;
    VictorSP leftMotor;
    VictorSP rightMotor;
    Encoder leftEncoder;
    Encoder rightEncoder;
    Trajectory left;
    Trajectory right;
    AHRS gyro;

    // Note, doesn't work... sorta

    @Override
    public void robotInit() {
        gyro = new AHRS(SPI.Port.kMXP);

        leftMotor = new VictorSP(0);
        rightMotor = new VictorSP(1);
        
        leftEncoder = new Encoder(0, 1, true);
        leftEncoder.setDistancePerPulse((6.0 * 0.0254 * Math.PI) / 213);

        rightEncoder = new Encoder(2, 3, false);
        rightEncoder.setDistancePerPulse((6.0 * 0.0254 * Math.PI) / 213);

        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 12.0, 2.0, 60.0);
        points = new Waypoint[] { new Waypoint(1, -1, 0), new Waypoint(0, 0, 0) };
        trajectory = Pathfinder.generate(points, config);
        modifier = new TankModifier(trajectory).modify(0.5);

        left = modifier.getLeftTrajectory();
        right = modifier.getRightTrajectory();

        leftFollow = new EncoderFollower(left);
        leftFollow.configureEncoder(0, 213, 0.1524);
        leftFollow.configurePIDVA(0.6, 0, 0, 1.0/12.0, 0);

        rightFollow = new EncoderFollower(right);
        rightFollow.configureEncoder(0, 213, 0.1524);
        rightFollow.configurePIDVA(0.6, 0, 0, 1.0/12.0, 0);

        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Path Following Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("GYRO??", gyro.isConnected());
        SmartDashboard.putNumber("Gyro", getHeading());
        SmartDashboard.putNumber("Turn Correction", turn);
        SmartDashboard.putNumber("Angle Delta", angleDelta);
        SmartDashboard.putNumber("Desired Heading", desiredHeading);
        SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
        SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
        SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    }

    int leftOffset = 0;
    int rightOffset = 0;

    @Override
    public void autonomousInit() {
        gyro.zeroYaw();

        leftEncoder.reset();
        rightEncoder.reset();

        leftOffset = -leftEncoder.get();
        rightOffset = -rightEncoder.get();
    }

    double desiredHeading = 0.0;
    double turn = 0.0;
    double angleDelta = 0.0;
    
    @Override
    public void autonomousPeriodic() {
        double gyroHeading = getHeading();
        desiredHeading = 180 - Pathfinder.r2d(leftFollow.getHeading());
        angleDelta = desiredHeading - gyroHeading;
        turn = 0.5 * (1.0 / 80.0) * angleDelta;
        // turn = 0.0;
        leftMotor.set(leftFollow.calculate(Math.abs(leftEncoder.get() + leftOffset)) + turn);
        rightMotor.set(-(rightFollow.calculate(Math.abs(rightEncoder.get() + rightOffset))) - turn);
    }

    private double getHeading(){
        double gyroHeading = gyro.getYaw();
        // if(gyroHeading < 0){
        //     gyroHeading += 360.0;
        // }
        return gyroHeading;
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testPeriodic() {
    }
}
