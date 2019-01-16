package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Drive extends Subsystem {

    public enum State {
        OPEN_LOOP, PATH_FOLLOWING, STOP
    }

    private static Drive instance;
    private State state;
    private final Victor left, right;
    private double leftMotorPower = 0.0;
    private double rightMotorPower = 0.0;
    private double deadband;

    private static final Waypoint[] TEST_POINTS = new Waypoint[] { new Waypoint(-2.0, -2.0, 0.0),
            new Waypoint(0.0, 0.0, 0.0) };

    private static final double MAX_VELOCITY = 0.25;
    private static final double MAX_ACCELERATION = 0.5;
    private static final double WHEEL_BASE_WIDTH = 0.56515;
    private static final double MAX_JERK = 0.1;
    private static final double LOOP_TIME = 0.05;
    private static final Trajectory.Config DRIVE_PATH_CONFIG = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH, LOOP_TIME, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
    private static final Trajectory TRAJECTORY = Pathfinder.generate(TEST_POINTS, DRIVE_PATH_CONFIG);
    private static final TankModifier MODIFIER = new TankModifier(TRAJECTORY).modify(WHEEL_BASE_WIDTH);

    private static final int ENCODER_TICKS_PER_REVOLUTION = 900;
    private static final double WHEEL_DIAMETER = 6.0 * 0.0254;
    public final Encoder leftEncoder;
    public final Encoder rightEncoder;
    private final EncoderFollower leftEncFollower;
    private final EncoderFollower rightEncFollower;

    private Drive() {
        left = new Victor(1);
        right = new Victor(0);
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);
        leftEncoder.setDistancePerPulse((WHEEL_DIAMETER * 3.141592) / ((double) ENCODER_TICKS_PER_REVOLUTION));
        rightEncoder.setDistancePerPulse((WHEEL_DIAMETER * 3.141592) / ((double) ENCODER_TICKS_PER_REVOLUTION));
        leftEncFollower = new EncoderFollower(MODIFIER.getLeftTrajectory());
        rightEncFollower = new EncoderFollower(MODIFIER.getRightTrajectory());
        leftEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / MAX_VELOCITY, 0);
        rightEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / MAX_VELOCITY, 0);
    }

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Drive state: ", state.toString());
        SmartDashboard.putNumber("Left Encoder: ", leftEncoder.getDistance());
        SmartDashboard.putNumber("Left Rate: ", leftEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder: ", rightEncoder.getDistance());
        SmartDashboard.putNumber("Right Rate: ", rightEncoder.getRate());
    }

    @Override
    public void stop() {
        pathConfigured = false;
        state = State.STOP;
        left.set(0.0);
        right.set(0.0);
    }

    public void arcadeDrive(double x, double zRotation) {
        arcadeDrive(x, zRotation, false);
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        if (this.state != State.OPEN_LOOP) {
            pathConfigured = false;
            this.state = State.OPEN_LOOP;
        }

        xSpeed = applyDeadband(limit(xSpeed), deadband);
        zRotation = applyDeadband(limit(zRotation), deadband);

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            if (zRotation >= 0.0) {
                leftMotorPower = maxInput;
                rightMotorPower = xSpeed - zRotation;
            } else {
                leftMotorPower = xSpeed + zRotation;
                rightMotorPower = maxInput;
            }
        } else {
            if (zRotation >= 0.0) {
                leftMotorPower = xSpeed + zRotation;
                rightMotorPower = maxInput;
            } else {
                leftMotorPower = maxInput;
                rightMotorPower = xSpeed - zRotation;
            }
        }
    }

    public void tankDrive(double leftPower, double rightPower) {
        tankDrive(leftPower, rightPower, true);
    }

    public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
        if (this.state != State.OPEN_LOOP) {
            pathConfigured = false;
            this.state = State.OPEN_LOOP;
        }
        leftMotorPower = applyDeadband(limit(leftPower), deadband);
        rightMotorPower = applyDeadband(limit(leftPower), deadband);
        if (squareInputs) {
            leftMotorPower = Math.copySign(leftMotorPower * leftMotorPower, leftPower);
            rightMotorPower = Math.copySign(rightMotorPower * rightMotorPower, rightMotorPower);
        }
    }

    private double limit(double val) {
        if (val > 1.0) {
            return 1.0;
        } else if (val < -1.0) {
            return -1.0;
        }
        return val;
    }

    boolean pathConfigured = false;

    public void configureTestPathFollow() {
        this.state = State.PATH_FOLLOWING;
        leftEncFollower.configureEncoder(leftEncoder.getRaw(), ENCODER_TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
        rightEncFollower.configureEncoder(rightEncoder.getRaw(), ENCODER_TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
        pathConfigured = true;
    }

    @Override
    public void onLoop(double timestamp) {
        switch (state) {
        case OPEN_LOOP:
            left.set(leftMotorPower);
            right.set(-rightMotorPower);
            break;
        case PATH_FOLLOWING:
            double leftOutput = leftEncFollower.calculate(leftEncoder.get());
            double rightOutput = -rightEncFollower.calculate(rightEncoder.get());
            left.set(leftOutput);
            right.set(rightOutput);
            break;
        case STOP:
            stop();
            break;
        }
    }

}