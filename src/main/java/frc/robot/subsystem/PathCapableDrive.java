package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

// NOTE: STILL VERY EXPERIMENTAL. DO NOT TRUST.
public class PathCapableDrive extends Subsystem {
    // Constants
    private static final int PULSES_PER_REVOLUTION = 213;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_DIAMETER = 6.0 * INCHES_TO_METERS;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
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

    // State
    private enum State {
        OPEN_LOOP, PATH_FOLLOWING_INIT, PATH_FOLLOWING, STOPPED
    }

    // Hardware
    private final VictorSP leftMotor;
    private final VictorSP rightMotor;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final AHRS gyro;

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
    private State state = State.STOPPED;
    private int leftOffset = 0;
    private int rightOffset = 0;
    private double desiredHeading = 0.0;
    private double turn = 0.0;
    private double angleDelta = 0.0;
    private double leftPercent = 0.0;
    private double rightPercent = 0.0;

    // Instance
    private static PathCapableDrive instance;

    private PathCapableDrive() {
        // NavX
        gyro = new AHRS(SPI.Port.kMXP);

        // Left and right side of drive base
        leftMotor = new VictorSP(0);
        rightMotor = new VictorSP(1);

        // Encoders on drive base, inverts the left side
        leftEncoder = new Encoder(0, 1, true);
        leftEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);

        rightEncoder = new Encoder(2, 3, false);
        rightEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);

        // Setup
        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, DT,
                MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
        points = new Waypoint[] { new Waypoint(1, -1, 0), new Waypoint(0, 0, 0) };
        trajectory = Pathfinder.generate(points, config);
        configurePath(trajectory);
    }

    private void configurePath(Trajectory trajectory) {
        modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);

        left = modifier.getLeftTrajectory();
        right = modifier.getRightTrajectory();

        leftFollow = new EncoderFollower(left);
        leftFollow.configureEncoder(0, PULSES_PER_REVOLUTION, WHEEL_DIAMETER);
        leftFollow.configurePIDVA(FOLLOWER_KP, FOLLOWER_KI, FOLLOWER_KD, FOLLOWER_KV, FOLLOWER_KA);

        rightFollow = new EncoderFollower(right);
        rightFollow.configureEncoder(0, PULSES_PER_REVOLUTION, WHEEL_DIAMETER);
        rightFollow.configurePIDVA(FOLLOWER_KP, FOLLOWER_KI, FOLLOWER_KD, FOLLOWER_KV, FOLLOWER_KA);

    }

    public static PathCapableDrive getInstance() {
        if (instance == null) {
            instance = new PathCapableDrive();
        }
        return instance;
    }

    @Override
    public void outputTelemetry() {
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
    public void onLoop(double timestamp) {
        switch (state) {
        case OPEN_LOOP:
            writePercentsToDrive(leftPercent, rightPercent);
            break;
        case PATH_FOLLOWING_INIT:
            gyro.zeroYaw();
            leftEncoder.reset();
            rightEncoder.reset();
            leftOffset = -leftEncoder.get();
            rightOffset = -rightEncoder.get();
            state = State.PATH_FOLLOWING;
            break;
        case PATH_FOLLOWING:
            double gyroHeading = getHeading();
            desiredHeading = 180.0 - Pathfinder.r2d(leftFollow.getHeading());
            angleDelta = desiredHeading - gyroHeading;
            // 1.125 percent per 180 degrees of error
            turn = (3.0 / 180.0) * angleDelta;
            writePercentsToDrive(leftFollow.calculate(Math.abs(leftEncoder.get() + leftOffset)) + turn, -(rightFollow.calculate(Math.abs(rightEncoder.get() + rightOffset))) - turn);
            break;
        case STOPPED:
            stop();
            break;
        }
    }

    @Override
    public void stop() {
        state = State.STOPPED;
        writePercentsToDrive(0.0, 0.0);
    }

    public void setDrivePercentOutput(double left, double right) {
        if (state != State.OPEN_LOOP) {
            state = State.OPEN_LOOP;
        }
        this.leftPercent = left;
        this.rightPercent = right;
    }

    public void startPath(){
        state = State.PATH_FOLLOWING_INIT;
    }

    private void writePercentsToDrive(double left, double right) {
        leftMotor.set(left);
        rightMotor.set(left);
    }

    private double getHeading() {
        return gyro.getYaw();
    }

}