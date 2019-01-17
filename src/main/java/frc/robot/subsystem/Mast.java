package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Solenoid;



public class Mast extends Subsystem{
    private final Solenoid mast = new Solenoid(0);

    private static Mast instance;

    public void Liftmast() {
        mast.set(true);
    }


    @Override
    public void onLoop(double timestamp) {
    }

    @Override
    public void stop() {

    }

    @Override
    public void outputTelemetry() {
    }

    public static Mast getInstance() {
        if (instance == null) {
            instance = new Mast();
        }
        return instance;  
    }
}