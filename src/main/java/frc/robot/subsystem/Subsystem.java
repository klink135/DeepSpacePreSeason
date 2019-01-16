package frc.robot.subsystem;

public abstract class Subsystem {
    public void zeroSensors(){}

    public abstract void outputTelemetry();

    public abstract void stop();

    public abstract void onLoop(double timestamp);
}