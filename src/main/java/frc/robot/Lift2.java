package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Lift extends Subsystem {

    private double lastError = 0.0;
    private double integratedError = 0.0;

    /**
     * lowest val = 3.5 => offset = -3.5 max val = 12.5 => range is 9 scaling 2/9
     */

    private static final double kP = 12.0 * (1 / LIFT_POT_FULL_RANGE);
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private double lastTimestamp = Timer.getFPGATimestamp();

    private enum State {
        PID, MANUAL
    }

    private State state;
    private final VictorSP motor;
    private double manualPower = 0.0;
    private AnalogPotentiometer m_pot;
    private static Lift instance;
    private static final double absoluteTolerance = 0.01;

    private Lift() {
        state = State.MANUAL;
        motor = new VictorSP(3);
        m_pot = new AnalogPotentiometer(0, LIFT_POT_FULL_RANGE, LIFT_POT_OFFSET);
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
        SmartDashboard.putNumber("Lift pot value", m_pot.get());
        SmartDashboard.putNumber("Integrated Error * kI", integratedError * kI);
        SmartDashboard.putNumber("Error", lastError);
        SmartDashboard.putBoolean("Has been in tolerance", hasBeenInTolerance);
    }

    @Override
    public void stop() {

    }

    private double goalHeight = 0.0;

    public void setHeight(double height) {
        if(state != State.PID){
            state = State.PID;
        }
        if(height < Constants.LIFT_DISTANCE_FROM_GROUND){
            height = Constants.LIFT_DISTANCE_FROM_GROUND;
        } else if(height > LIFT_DISTANCE_FROM_GROUND + LIFT_MAX_HEIGHT){
            height = LIFT_DISTANCE_FROM_GROUND + LIFT_MAX_HEIGHT;
        }
        this.goalHeight = height;
        integratedError = 0;
    
    }

    public double getHeight(){
        return goalHeight;
    }

    boolean hasBeenInTolerance = false;
    double inToleranceValue = 0.0;

    @Override
    public void onLoop(double timestamp) {
        switch (state) {
        case MANUAL:
            motor.set(manualPower);
        case PID:
            double potVal = m_pot.get();
            double error = goalHeight - potVal;
            double derivError = lastError - error;
            integratedError += error * (timestamp - lastTimestamp);
            double output = kP * error + kD * derivError + kI * integratedError;
            if(Math.abs(error) < absoluteTolerance){
                hasBeenInTolerance = true;
                inToleranceValue = output;
            } else {
                hasBeenInTolerance = false;
            }

            if(hasBeenInTolerance){
                motor.set(inToleranceValue);
            } else {
                motor.set(output);
            }
            lastError = error;
            lastTimestamp = timestamp;
        }
    }

}
