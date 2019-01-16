package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;

public class Subframe extends Subsystem {

    private final static double REAR_LEFT_LIFT_MULTIPLIER = 0.75 / 0.85;
    private final static double REAR_RIGHT_LIFT_MULTIPLIER = 0.65 / 0.85;
    private final static double FRONT_LIFT_MULTIPLIER = 1.0;

    private final static double LIFT_ALL_DEFAULT_MAX_SPEED = 1.0;

    private final Spark frontLift;
    private final VictorSP rearRightLift;
    private final VictorSP rearLeftLift;
    private final VictorSP rearRightWheel;
    private final WPI_TalonSRX rearLeftWheel;

    private double frontLiftSpeed;
    private double rearRightLiftSpeed;
    private double rearLeftLiftSpeed;
    private double rearRightWheelSpeed;
    private double rearLeftWheelSpeed;

    private Subframe() {
        frontLift = new Spark(2);
        rearRightLift = new VictorSP(3);
        rearLeftLift = new VictorSP(1);
        rearRightWheel = new VictorSP(0);
        rearLeftWheel = new WPI_TalonSRX(0);
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
        frontLiftSpeed = 0;
        rearRightLiftSpeed = 0;
        rearLeftLiftSpeed = 0;
        frontLiftSpeed = 0;
        updatePowers();
    }

    @Override
    public void onLoop(double timestamp) {
        updatePowers();
    }

    private void updatePowers() {
        frontLift.set(frontLiftSpeed);
        rearRightLift.set(rearRightLiftSpeed);
        rearLeftLift.set(rearLeftLiftSpeed);
        rearRightWheel.set(rearRightWheelSpeed);
        rearLeftWheel.set(rearLeftWheelSpeed);
    }

    public void retractFrontLift() {
        frontLiftSpeed = LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void extendFrontLift() {
        frontLiftSpeed = -LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void retractRearLeftLift() {
        rearLeftLiftSpeed = -LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void extendRearLeftLift() {
        rearLeftLiftSpeed = LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void retractRearRightLift() {
        rearRightLiftSpeed = -LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void extendRearRightLift() {
        rearRightLiftSpeed = LIFT_ALL_DEFAULT_MAX_SPEED;
    }

    public void liftAll() {
        liftAll(LIFT_ALL_DEFAULT_MAX_SPEED);
    }

    public void liftAll(double power) {
        frontLiftSpeed = -power * FRONT_LIFT_MULTIPLIER;
        rearLeftLiftSpeed = power * REAR_LEFT_LIFT_MULTIPLIER;
        rearRightLiftSpeed = power * REAR_RIGHT_LIFT_MULTIPLIER;
    }

    public void retractAll() {
        retractAll(LIFT_ALL_DEFAULT_MAX_SPEED);
    }

    public void retractAll(double power) {
        liftAll(-power);
    }

    public void runWheels(double power) {
        runRearLeftWheel(power);
        runRearRightWheel(power);
    }

    public void runRearLeftWheel(double power) {
        rearLeftWheelSpeed = power;
    }

    public void runRearRightWheel(double power) {
        rearRightWheelSpeed = power;
    }
}