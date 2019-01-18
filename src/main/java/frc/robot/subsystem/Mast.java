package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mast extends Subsystem{
    private static Mast instance;
    private final Solenoid mast;
    private boolean isMastLifted = false;

    private Mast(){
        mast = new Solenoid(0);
    }

    public static Mast getInstance() {
        if (instance == null) {
            instance = new Mast();
        }
        return instance;  
    }

    public void liftMast() {
        isMastLifted = true;
        mast.set(isMastLifted);
    }


    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Mast Lifted", isMastLifted);
    }
}