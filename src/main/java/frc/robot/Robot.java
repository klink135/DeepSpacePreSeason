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
import frc.robot.subsystem.Vision;
import frc.robot.subsystem.Vision.Camera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

// TODO remove all the junk from this
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
  private final Vision vision = Vision.getInstance();
  private final Spark gripperWrist = new Spark(2);
  public final Joystick m_operatorJoystick = new Joystick(1);

  // TODO add gripper/gripperWrist to subsystem
  // TODO add SuperJoystick extension of driver joystick
  // TODO clean out old code

  // private final DoubleSolenoid m_gripper = new DoubleSolenoid(1,2);
  public final Joystick driverJoystick = new Joystick(0);
  // private Lift lift = Lift.getInstance();
  private OperatorGamepad operatorGamepad = OperatorGamepad.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
    chooser.addOption("Path Following Auto", CUSTOM_AUTO);
    SmartDashboard.putData("Auto choices", chooser);

    // lift.liftinit();
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
    boolean buttonZero = driverJoystick.getRawButton(1);
    if (buttonZero) {
      if (!buttonZeroPrevious) {
        if (frontCameraUpper) {
          frontCameraUpper = false;
          vision.focus(Camera.REAR);
        } else {
          frontCameraUpper = true;
          vision.focus(Camera.FRONT);
        }
      }
    }
    buttonZeroPrevious = buttonZero;

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
