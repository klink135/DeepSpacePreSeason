/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.CameraServer;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;

import edu.wpi.cscore.UsbCamera;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(new VictorSP(1), new VictorSP(0));  
  private final Joystick m_stick = new Joystick(0);  
  private static final int NUMBER_OF_CAMERAS = 2;

  private static final int FRONT_CAMERA_PORT = 0;
  private static final int REAR_CAMERA_PORT = 1;

  private static final String FRONT_CAMERA_NAME = "front";
  private static final String REAR_CAMERA_NAME = "rear";
  HashMap<String, UsbCamera> cameraList = new HashMap<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    cameraList.put(FRONT_CAMERA_NAME, CameraServer.getInstance().startAutomaticCapture(FRONT_CAMERA_PORT));
    cameraList.put(REAR_CAMERA_NAME, CameraServer.getInstance().startAutomaticCapture(REAR_CAMERA_PORT));
    for(UsbCamera usbCamera : cameraList.values()){
      usbCamera.setResolution(320, 240);
      usbCamera.setFPS(20);
      usbCamera.setExposureAuto();
    }


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  private static void lowerResolution(UsbCamera camera){
    camera.setResolution(32, 24);
    camera.setFPS(30);
  }

  private static void upperResolution(UsbCamera camera){
    camera.setResolution(640, 480);
    camera.setFPS(5);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
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
    boolean buttonZero = m_stick.getRawButton(1);
    if(buttonZero){
      if(!buttonZeroPrevious){
        if(frontCameraUpper){
          frontCameraUpper = false;
        lowerResolution(cameraList.get(FRONT_CAMERA_NAME));
        upperResolution(cameraList.get(REAR_CAMERA_NAME));
      } else {
        frontCameraUpper = true;
        upperResolution(cameraList.get(FRONT_CAMERA_NAME));
        lowerResolution(cameraList.get(REAR_CAMERA_NAME));
      }
    } 
    }

    buttonZeroPrevious = buttonZero;
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
