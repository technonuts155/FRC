// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Time;
import java.util.TimerTask;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
    reverse,
    stop
  }

  AutoStates currentState;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Shooter shooter = new Shooter();
  Drive drive = new Drive();
  Climb climb = new Climb();

  // Don't worry about it
  double startTime = 0;
  double timeStamp = 0;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Initalized the PixyDrivePID
    drive.initializePixy();

    // Drive motors need to be inverted manually now
    drive.invertLeftDriveMotors();

    // Initialize shooter Encoder and PID
    shooter.initalizeEncoder();
    shooter.initPID();

    // Initialize drive encoders
    drive.initializeEncoders();
  
    // Start climber servo in unlocked position
    climb.setLock(Climb.Lock.unlocked);
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
    SmartDashboard.putNumber("Left Encoder distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder distance", drive.getRightEncoderDistance());
    SmartDashboard.putBoolean("Ball loaded", shooter.ballIsLoaded());
    SmartDashboard.putBoolean("At speed", shooter.isUpToSpeed());
    SmartDashboard.putNumber("ColorSensor IR", shooter.getColorSensorIR());
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
    SmartDashboard.putBoolean("Lower Climb Limit", climb.getLimitLower());
    SmartDashboard.putBoolean("Upper Climb Limit", climb.getLimitUpper());
    drive.updateEncoderPIDValues();
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

    startTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    SmartDashboard.putNumber("Match time", Timer.getFPGATimestamp() - startTime);
    SmartDashboard.putString("Auto State", currentState.toString());

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
        if (Timer.getFPGATimestamp() - startTime > 1.5 && Timer.getFPGATimestamp() - startTime < 3) {
          currentState = AutoStates.collect;
        }
        break;

      case collect:
        // Actions in case
        drive.pixyAutopilot(-.65);
        shooter.intakeIn();
        shooter.indexForwardsSlow();
        shooter.setShooterRPM(Shooter.RPM.kStop);

        // Condition for changing cases
        if(shooter.ballIsLoaded() == true) {
          timeStamp = Timer.getFPGATimestamp();
          currentState = AutoStates.stop;
        }
        break;
      
      case reverse:
        // Actions in case
        drive.encoderPIDDrive();

        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.intakeStop();
        shooter.indexStop();

        // Condition for changing cases
        if (Math.abs(drive.getRightEncoderDistance()) < 5 && Math.abs(drive.getLeftEncoderDistance()) < 5) {
          timeStamp = Timer.getFPGATimestamp();
          currentState = AutoStates.shoot;
          //Change to stop if left encoder does not work
        }
        break;

      case stop:
        // Actions in case
        drive.setLeftMotors(0);
        drive.setRightMotors(0);
        shooter.indexStop();
        shooter.intakeStop();
        shooter.setShooterRPM(Shooter.RPM.kStop);

        // Condition for changing cases
        if (Timer.getFPGATimestamp() - timeStamp > .5) {
          currentState = AutoStates.reverse;
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
    } else if(OI.intakeIn() || shooter.isUpToSpeed() || OI.pixyAutopilot()) {
      shooter.intakeIn();
    } else {
      shooter.intakeStop();
    }

    // Indexer control
    if(OI.moveIndexUp() || shooter.isUpToSpeed()) {
      shooter.indexForwards();
    } else if (shooter.getIntakeSpeed() < -.5 && shooter.ballIsLoaded() == false) {
      shooter.indexForwardsSlow();
    } else if (OI.moveIndexDown()) {
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

    // Climber control
    if (OI.climbUp() && climb.getLimitUpper() == false && climb.getLock() < 10) {
      climb.setClimbUp();
    } else if (OI.climbDown() && climb.getLimitLower() == false && climb.getLock() < 10) {
      climb.setClimbDown();
    } else {
      climb.setClimbStop();
    }

    if (OI.climbLock()) {
      climb.setLock(Climb.Lock.locked);
    }
    if (OI.climbUnlock()) {
      climb.setLock(Climb.Lock.unlocked);
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
    if (OI.driverController.getAButton()) {
      shooter.setShooterRPM(Shooter.RPM.kSendIt);
    } else {
      shooter.setShooterRPM(Shooter.RPM.kStop);
    }

    System.out.println(shooter.isUpToSpeed());
    System.out.println(shooter.getShooterRPM());
  }
}
