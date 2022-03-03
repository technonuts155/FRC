// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  enum AutoStates {
    shoot,
    collect,
    reverse
  }

  AutoStates currentState;

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
    // Set default starting state for autonomous
    currentState = AutoStates.shoot;

    // Initalized the PixyDrivePID
    drive.initializePixy();

    // Drive motors need to be inverted manually now
    drive.invertLeftDriveMotors();

    // Initialize shooter Encoder and PID
    shooter.initalizeEncoder();
    shooter.initPID();
    drive.initializeEncoders();
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Block block = drive.getTargetBlock();
    SmartDashboard.putBoolean("Target aquired", block != null);
    SmartDashboard.putNumber("Left Encoder distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder distance", drive.getRightEncoderDistance());
  }

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
    // Make sure autonomous starts by shooting
    currentState = AutoStates.shoot;

    // Zero the encoders
    drive.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (currentState) {
      case shoot:
        // Actions in case
        shooter.setShooterRPM(Shooter.RPM.kHigh);

        if (shooter.isUpToSpeed()) {
          shooter.indexForwards();
        } else {
          shooter.indexStop();
        }

        shooter.intakeStop();
        drive.setLeftMotors(0);
        drive.setRightMotors(0);
        
        // Condition for changing cases
        if (DriverStation.getMatchTime() > 4 && DriverStation.getMatchTime() < 6) {
          currentState = AutoStates.collect;
        }
        break;

      case collect:
        // Actions in case
        drive.pixyAutopilot(.5);
        shooter.intakeIn();
        shooter.indexForwards();

        // Condition for changing cases
        //true is subject to change, could be false
        if(shooter.getBeamBreak() == true) {
          currentState = AutoStates.reverse;
        }
        break;
      
      case reverse:
        // Actions in case
        if (drive.getLeftEncoderDistance() > 4) {
          drive.setLeftMotors(-0.5);
        } else {
          drive.setLeftMotors(0);
        }

        if (drive.getRightEncoderDistance() > 4) {
          drive.setRightMotors(-0.5);
        } else {
          drive.setRightMotors(0);
        }

        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.intakeStop();
        shooter.indexStop();

        // Condition for changing cases
        if (drive.getLeftEncoderDistance() < 4 && drive.getRightEncoderDistance() < 4) {
          currentState = AutoStates.shoot;
        }
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
      drive.pixyAutopilot(OI.driveThrottle());
    } else {
      drive.XboxDrive();
    }

    // Intake Control
    if(OI.intakeOut()) {
      shooter.intakeOut();
    } else if(OI.intakeIn() || shooter.isUpToSpeed()) {
      shooter.intakeIn();
    } else {
      shooter.intakeStop();
    }

    // Indexer control
    if(OI.moveIndexUp() || shooter.isUpToSpeed()) {
      shooter.indexForwards();
    } else if(OI.moveIndexDown()) {
      shooter.indexBackwards();
    } else {
      shooter.indexStop();
    }
 
    // Shooter control
    if (OI.shooterManualOverride()) {
      shooter.setShooterPercentOutput(OI.shooterThrottle());
    } else {
      if (OI.shootHigh()) {
        shooter.setShooterRPM(Shooter.RPM.kHigh); 
      } else if (OI.shootLow()) {
        shooter.setShooterRPM(Shooter.RPM.kLow);
      } else {
        shooter.setShooterRPM(Shooter.RPM.kStop);;
      }
    }

    SmartDashboard.putBoolean("Shooter at setpoint", shooter.isUpToSpeed());
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
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

  }
}
