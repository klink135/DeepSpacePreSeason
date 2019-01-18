/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

public class PathRobot extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "Path Follpwing Test";
    private final SendableChooser<String> chooser = new SendableChooser<>();

    // Constants
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_DIAMETER = 6.0 * INCHES_TO_METERS;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private static final int PULSES_PER_REVOLUTION = 213;
    private static final double WHEELBASE_WIDTH = 0.56515;
    private static final double MAX_VELOCITY = 12.0;
    private static final double MAX_ACCELERATION = 2.0;
    private static final double MAX_JERK = 60.0;
    private static final double DT = 0.02;
    private static final double FOLLOWER_KP = 0.6;
    private static final double FOLLOWER_KI = 0.0;
    private static final double FOLLOWER_KD = 0.0;
    private static final double FOLLOWER_KV = 1.0 / MAX_VELOCITY;
    private static final double FOLLOWER_KA = 0.0;

    // Hardware
    private VictorSP leftMotor;
    private VictorSP rightMotor;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private AHRS gyro;

    // Pathfinder
    private Trajectory.Config config;
    private Waypoint[] points;
    private Trajectory trajectory;
    private Trajectory left;
    private Trajectory right;
    private TankModifier modifier;
    private EncoderFollower leftFollow;
    private EncoderFollower rightFollow;
    
    // Instance variables
    private int leftOffset = 0;
    private int rightOffset = 0;
    private double desiredHeading = 0.0;
    private double turn = 0.0;
    private double angleDelta = 0.0;

    @Override
    public void robotInit() {
        // NavX
        gyro = new AHRS(SPI.Port.kMXP);

        // Left and right side of drive base
        leftMotor = new VictorSP(0);
        rightMotor = new VictorSP(1);
<<<<<<< HEAD
        
        leftEncoder = new Encoder(0, 1, false);
        leftEncoder.setDistancePerPulse((6.0 * 0.0254 * Math.PI) / 213);

        rightEncoder = new Encoder(2, 3, true);
        rightEncoder.setDistancePerPulse((6.0 * 0.0254 * Math.PI) / 213);

        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 12.0, 2.0, 60.0);
        points = new Waypoint[] { new Waypoint(2, -2, 0), new Waypoint(0, 0, 0) };
=======

        // Encoders on drive base, inverts the left side
        leftEncoder = new Encoder(0, 1, true);
        leftEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);

        rightEncoder = new Encoder(2, 3, false);
        rightEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);

        // Setup
        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, DT,
                MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
        points = new Waypoint[] { new Waypoint(1, -1, 0), new Waypoint(0, 0, 0) };
>>>>>>> 8e233ad577bb9d0327b94528faa65cdde2404f52
        trajectory = Pathfinder.generate(points, config);
        modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);

        left = modifier.getLeftTrajectory();
        right = modifier.getRightTrajectory();

        leftFollow = new EncoderFollower(left);
<<<<<<< HEAD
        leftFollow.configureEncoder(0, 213, 0.1524);
        leftFollow.configurePIDVA(0.9, 0, .4, 1.0/12.0, 0);

        rightFollow = new EncoderFollower(right);
        rightFollow.configureEncoder(0, 213, 0.1524);
        rightFollow.configurePIDVA(0.9, 0, .4, 1.0/12.0, 0);
=======
        leftFollow.configureEncoder(0, PULSES_PER_REVOLUTION, WHEEL_DIAMETER);
        leftFollow.configurePIDVA(FOLLOWER_KP, FOLLOWER_KI, FOLLOWER_KD, FOLLOWER_KV, FOLLOWER_KA);

        rightFollow = new EncoderFollower(right);
        rightFollow.configureEncoder(0, PULSES_PER_REVOLUTION, WHEEL_DIAMETER);
        rightFollow.configurePIDVA(FOLLOWER_KP, FOLLOWER_KI, FOLLOWER_KD, FOLLOWER_KV, FOLLOWER_KA);
>>>>>>> 8e233ad577bb9d0327b94528faa65cdde2404f52

        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Path Following Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("Gyro Connected", gyro.isConnected());
        SmartDashboard.putNumber("Gyro", getHeading());
        SmartDashboard.putNumber("Turn Correction", turn);
        SmartDashboard.putNumber("Angle Delta", angleDelta);
        SmartDashboard.putNumber("Desired Heading", desiredHeading);
        SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
        SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
        SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    }

    @Override
    public void autonomousInit() {
        gyro.zeroYaw();

        leftEncoder.reset();
        rightEncoder.reset();

        leftOffset = -leftEncoder.get();
        rightOffset = -rightEncoder.get();
    }

    @Override
    public void autonomousPeriodic() {
        double gyroHeading = getHeading();
<<<<<<< HEAD
        desiredHeading = 180 - Pathfinder.r2d(leftFollow.getHeading()); 
        angleDelta = desiredHeading - gyroHeading;
        turn = 1.6 * (1.0 / 80.0) * angleDelta;
        //turn = 0.0;
=======
        desiredHeading = 180.0 - Pathfinder.r2d(leftFollow.getHeading());
        angleDelta = desiredHeading - gyroHeading;
        // 1.125 percent per 180 degrees of error
        turn = (1.125 / 180.0) * angleDelta;
>>>>>>> 8e233ad577bb9d0327b94528faa65cdde2404f52
        leftMotor.set(leftFollow.calculate(Math.abs(leftEncoder.get() + leftOffset)) + turn);
        rightMotor.set(-(rightFollow.calculate(Math.abs(rightEncoder.get() + rightOffset))) - turn);
    }

    private double getHeading() {
        return gyro.getYaw();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testPeriodic() {
    }
}
