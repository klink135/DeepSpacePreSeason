package frc.robot.subsystem;

import java.util.HashMap;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Vision extends Subsystem {
    private static Vision instance;

    private enum Camera {
        FRONT(0), REAR(1);

        Camera(int cameraPort) {
            this.cameraPort = cameraPort;
        }

        private int cameraPort;

        public int getCameraPort() {
            return cameraPort;
        }
    }

    private HashMap<Camera, UsbCamera> cameras = new HashMap<>();

    private Vision() {
        for (Camera camera : Camera.values()) {
            cameras.put(camera, CameraServer.getInstance().startAutomaticCapture(camera.getCameraPort()));
        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void onLoop(double timestamp) {
        // Do nothing
    }

    public void focus(Camera cameraToFocus) {
        upperResolution(cameras.get(cameraToFocus));
        for (Camera camera : Camera.values()) {
            if (camera != cameraToFocus) {
                lowerResolution(cameras.get(camera));
            }
        }
    }

    private static void lowerResolution(/*Nullable*/UsbCamera camera) {
        if (camera != null) {
            camera.setResolution(32, 24);
            camera.setFPS(5);
        }
    }

    private static void upperResolution(/*Nullable*/ UsbCamera camera) {
        if (camera != null) {
            camera.setResolution(320, 240);
            camera.setFPS(15);
        }
    }

}