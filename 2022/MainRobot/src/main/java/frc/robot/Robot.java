// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Shooter shooter = new Shooter();
  Drive drive = new Drive();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Initalized the PixyDrivePID
    drive.initializePixy();

    // Right Drive motors need to be inverted manually now
    drive.invertRightDriveMotors();

    // Initialize shooter PID and Encoder
    shooter.initPID();
    shooter.initalizeEncoder();
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Drive control
    if (OI.pixyAutopilot() == true) {
      drive.pixyAutopilot();
    } else {
      drive.XboxDrive();
      drive.resetPID();     // Reset PID during manual drive to prevent integral build-up
    }

    // Cargo intake control
    if (OI.manualForwardsIntake() == true || OI.pixyAutopilot() == true) {
      shooter.intakeForwards();
    } else if (OI.manualReverseIntake() == true) {
      shooter.intakeReverse();
    } else {
      shooter.intakeStop();
    }

    // Cargo index control
    if (shooter.getBeamBreak() == true && shooter.getIntakeSpeed() == 1) {
      shooter.indexForwards();
    } else if (shooter.isUpToSpeed()) {
      shooter.indexForwards();
    } else if (OI.moveIndexDown()) {
      shooter.indexBackwards();
    } else if (OI.moveIndexUp()) {
      shooter.indexForwards();
    } else {
      shooter.indexStop();
    }

    // Shooter control
    // Shooter speed is controlled by operator when override button is held, PIDController otherwise
    if (OI.shooterManualOverride()) {
      shooter.setShooterPercentOutput(OI.shooterThrottle());
    } else {
      if (OI.shootHigh()) {
        shooter.setShooterRPM(Shooter.RPM.kHigh); 
      } else if (OI.shootLow()) {
        shooter.setShooterRPM(Shooter.RPM.kLow);
      } else {
        shooter.setShooterRPM(Shooter.RPM.kStop);
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(OI.manualForwardsIntake()) {
      shooter.intakeForwards();
    } else if(OI.manualReverseIntake()) {
      shooter.intakeReverse();
    } else {
      shooter.intakeStop();
    }

    if(OI.moveIndexUp()) {
      shooter.indexForwards();
    } else if(OI.moveIndexDown()) {
      shooter.indexBackwards();
    } else {
      shooter.indexStop();
    }
 
    shooter.setShooterPercentOutput(OI.shooterThrottle());

    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());

    drive.XboxDrive();

  }
}
