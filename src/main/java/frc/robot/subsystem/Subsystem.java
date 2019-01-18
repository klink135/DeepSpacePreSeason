package frc.robot.subsystem;

public abstract class Subsystem {
    public void zeroSensors(){}

    public abstract void onLoop(double timestamp);
    
    public abstract void outputTelemetry();
    
    public abstract void stop();
}