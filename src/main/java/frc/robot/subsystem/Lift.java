package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.INCHES_TO_METERS;

public class Lift extends Subsystem {
    public static final double DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS = 28.0 * INCHES_TO_METERS;
    public static final double HATCH_DISENGAGE_DISTANCE = 5.0 * INCHES_TO_METERS;
    public static final double LIFT_DISTANCE_FROM_GROUND = 22 * INCHES_TO_METERS; // meters
    public static final double LIFT_MAX_HEIGHT = 68 * INCHES_TO_METERS; // meters
    public static final double LIFT_POT_MIN_POSITION_VOLTAGE = 0.723;
    public static final double LIFT_POT_MAX_POSITION_VOLTAGE = 0.339;
    public static final double LIFT_POT_OFFSET = -LIFT_MAX_HEIGHT
            * (LIFT_POT_MIN_POSITION_VOLTAGE / (LIFT_POT_MAX_POSITION_VOLTAGE - LIFT_POT_MIN_POSITION_VOLTAGE))
            + LIFT_DISTANCE_FROM_GROUND; // meters/volt
    public static final double LIFT_POT_FULL_RANGE = LIFT_MAX_HEIGHT
            * (1.0 / (LIFT_POT_MAX_POSITION_VOLTAGE - LIFT_POT_MIN_POSITION_VOLTAGE));
    public static final double LIFT_HATCH_POSITION_LOW = (26 + (3.0 / 8.0)) * INCHES_TO_METERS;
    public static final double LIFT_HATCH_POSITION_MID = LIFT_HATCH_POSITION_LOW
            + DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS;
    public static final double LIFT_HATCH_POSITION_HIGH = LIFT_HATCH_POSITION_MID
            + DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS;

    private static final double kP = 12.0 * (1 / LIFT_POT_FULL_RANGE);
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double absoluteTolerance = 0.01;

    private enum State {
        PID, MANUAL
    }

    private State state;
    private double lastError = 0.0;
    private double integratedError = 0.0;
    private double lastTimestamp = Timer.getFPGATimestamp();
    private double manualPower = 0.0;
    private double goalHeight = 0.0;
    private boolean hasBeenInTolerance = false;
    private double inToleranceValue = 0.0;

    private final VictorSP motor;
    private AnalogPotentiometer pot;

    private static Lift instance;

    private Lift() {
        state = State.MANUAL;
        motor = new VictorSP(3);
        pot = new AnalogPotentiometer(0, LIFT_POT_FULL_RANGE, LIFT_POT_OFFSET);
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    public void manuallyMove(double power) {
        if (state != State.MANUAL) {
            state = State.MANUAL;
        }
        manualPower = power;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Lift pot value", pot.get());
        SmartDashboard.putNumber("Integrated Error * kI", integratedError * kI);
        SmartDashboard.putNumber("Error", lastError);
        SmartDashboard.putBoolean("Has been in tolerance", hasBeenInTolerance);
    }

    @Override
    public void stop() {

    }

    public void setHeight(double height) {
        if (state != State.PID) {
            state = State.PID;
        }
        if (height < LIFT_DISTANCE_FROM_GROUND) {
            height = LIFT_DISTANCE_FROM_GROUND;
        } else if (height > LIFT_DISTANCE_FROM_GROUND + LIFT_MAX_HEIGHT) {
            height = LIFT_DISTANCE_FROM_GROUND + LIFT_MAX_HEIGHT;
        }
        this.goalHeight = height;
        integratedError = 0;

    }

    public double getHeight() {
        return goalHeight;
    }

    @Override
    public void onLoop(double timestamp) {
        switch (state) {
        case MANUAL:
            motor.set(manualPower);
        case PID:
            double potVal = pot.get();
            double error = goalHeight - potVal;
            double derivError = lastError - error;
            integratedError += error * (timestamp - lastTimestamp);
            double output = kP * error + kD * derivError + kI * integratedError;
            if (Math.abs(error) < absoluteTolerance) {
                hasBeenInTolerance = true;
                inToleranceValue = output;
            } else {
                hasBeenInTolerance = false;
            }

            if (hasBeenInTolerance) {
                motor.set(inToleranceValue);
            } else {
                motor.set(output);
            }

            lastError = error;
            lastTimestamp = timestamp;
        }
    }

}
