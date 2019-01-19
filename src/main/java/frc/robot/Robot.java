/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.hid.OperatorGamepad;
import frc.robot.subsystem.Drive;
import frc.robot.subsystem.Lift;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import java.util.HashMap;
// TODO remove all the junk from this
// TODO add vision subsystem
// TODO add subsystem manager
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String DEFAULT_AUTO = "Default";
  private static final String CUSTOM_AUTO = "Path Following Test";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();
  // TODO private final DifferentialDrive robotDrive = new DifferentialDrive(new
  // VictorSP(1), new VictorSP(0));
  private final Drive robotDrive = Drive.getInstance();
  private final Spark gripperWrist = new Spark(2);
  public final Joystick m_operatorJoystick = new Joystick(1);

  // TODO add gripper/gripperWrist to subsystem
  // TODO add vision to its own class
  // TODO add SuperJoystick extension of driver joystick
  // TODO clean out old code

  // private final DoubleSolenoid m_gripper = new DoubleSolenoid(1,2);
  public final Joystick driverJoystick = new Joystick(0);
  // private static final int NUMBER_OF_CAMERAS = 5;
  // private static final int FRONT_CAMERA_PORT = 0;
  // private static final String FRONT_CAMERA_NAME = "front";
  // private static final int REAR_CAMERA_PORT = 1;
  // private static final String REAR_CAMERA_NAME = "rear";

  /*
   * private static final int LEFT_CAMERA_PORT = 2; private static final int
   * RIGHT_CAMERA_PORT = 3; private static final int PIXYVIEW_CAMERA_PORT = 4;
   * 
   * private static final String LEFT_CAMERA_NAME = "left"; private static final
   * String RIGHT_CAMERA_NAME = "right"; private static final String
   * PIXYVIEW_CAMERA_NAME = "pixy";
   */
  // HashMap<String, UsbCamera> cameraList = new HashMap<>();
  // private Lift lift = Lift.getInstance();
  private OperatorGamepad operatorGamepad = OperatorGamepad.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // cameraList.put(FRONT_CAMERA_NAME,
    // CameraServer.getInstance().startAutomaticCapture(FRONT_CAMERA_PORT));
    // cameraList.put(REAR_CAMERA_NAME,
    // CameraServer.getInstance().startAutomaticCapture(REAR_CAMERA_PORT));
    /*
     * cameraList.put(LEFT_CAMERA_NAME,
     * CameraServer.getInstance().startAutomaticCapture(LEFT_CAMERA_PORT));
     * cameraList.put(RIGHT_CAMERA_NAME,
     * CameraServer.getInstance().startAutomaticCapture(RIGHT_CAMERA_PORT));
     * cameraList.put(PIXYVIEW_CAMERA_NAME,
     * CameraServer.getInstance().startAutomaticCapture(PIXYVIEW_CAMERA_PORT));
     */
    // for (UsbCamera usbCamera : cameraList.values()) {
    // usbCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 15);
    // // usbCamera.setResolution(80, 60);
    // // usbCamera.setFPS(5);
    // // usbCamera.setExposureManual(20);
    // usbCamera.setExposureAuto();
    // }

    chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
    chooser.addOption("Path Following Auto", CUSTOM_AUTO);
    SmartDashboard.putData("Auto choices", chooser);

    // lift.liftinit();
  }

  private static void lowerResolution(UsbCamera camera) {
    camera.setResolution(32, 24);
    camera.setFPS(5);
  }

  private static void upperResolution(UsbCamera camera) {
    camera.setResolution(320, 240);
    camera.setFPS(15);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    robotDrive.leftEncoder.reset();
    robotDrive.rightEncoder.reset();
    robotDrive.configureTestPathFollow();


    // autoSelected = SmartDashboard.getString("Auto Selector", DEFAULT_AUTO);
    System.out.println("Auto selected: " + autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
    case CUSTOM_AUTO:
      robotDrive.onLoop(Timer.getFPGATimestamp());
      robotDrive.outputTelemetry();
      break;
    case DEFAULT_AUTO:
    default:
      break;

    }
  }

  private boolean buttonZeroPrevious = false;
  private boolean frontCameraUpper = false;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // boolean buttonZero = driverJoystick.getRawButton(1);
    // if (buttonZero) {
    // if (!buttonZeroPrevious) {
    // if (frontCameraUpper) {
    // frontCameraUpper = false;
    // lowerResolution(cameraList.get(FRONT_CAMERA_NAME));
    // upperResolution(cameraList.get(REAR_CAMERA_NAME));
    // } else {
    // frontCameraUpper = true;
    // upperResolution(cameraList.get(FRONT_CAMERA_NAME));
    // lowerResolution(cameraList.get(REAR_CAMERA_NAME));
    // }
    // }
    // }
    // buttonZeroPrevious = buttonZero;

    // SmartDashboard.putNumber("pot", lift2.m_pot.get());

    // if (operatorGamepad.liftToMidHatch()) {
    // lift.setHeight(Lift.LIFT_HATCH_POSITION_MID);
    // } else if (operatorGamepad.liftToLowHatch()) {
    // lift.setHeight(Lift.LIFT_HATCH_POSITION_LOW);
    // } else if (operatorGamepad.liftToHighHatch()) {
    // lift.setHeight(Lift.LIFT_HATCH_POSITION_HIGH);
    // } else if (operatorGamepad.disengageButtonTapped()) {
    // lift.setHeight(lift.getHeight() - Lift.HATCH_DISENGAGE_DISTANCE);
    // } else {
    // lift.manuallyMove(0.0);
    // }
    // lift.onLoop(Timer.getFPGATimestamp());
    // lift.outputTelemetry();

    robotDrive.arcadeDrive(driverJoystick.getY(), driverJoystick.getX());
    robotDrive.onLoop(Timer.getFPGATimestamp());
    robotDrive.outputTelemetry();
    gripperWrist.set(operatorGamepad.getGripperPower());
    /*
     * if(m_operatorJoystick.getRawButton(5)){
     * m_gripper.set(DoubleSolenoid.Value.kForward); }else
     * if(m_operatorJoystick.getRawButton(6)){
     * m_gripper.set(DoubleSolenoid.Value.kReverse); }
     */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
