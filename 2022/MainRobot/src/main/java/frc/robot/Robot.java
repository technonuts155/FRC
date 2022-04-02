// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    stop,
    aim
  }

  AutoStates currentState;

  Shooter shooter = new Shooter();
  Drive drive = new Drive();
  Climb climb = new Climb();

  // Variables to hold time stamps during Autonomous
  double startTime = 0;
  double timeStamp = 0;

  // Debouncer for climber. Gives the lock a half second to move before the lift begins movement
  Debouncer climbDebouncer = new Debouncer(.5, DebounceType.kRising);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    climb.lock();
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
    SmartDashboard.putBoolean("Ball loaded", shooter.ballIsLoaded());
    SmartDashboard.putBoolean("At speed", shooter.isUpToSpeed());
    SmartDashboard.putNumber("ColorSensor IR", shooter.getColorSensorIR());
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("Left Encoder Distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance", drive.getRightEncoderDistance());
    drive.displayMotorControllerInputs();
    drive.updateHubPIDValues();
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

    // Get a system timestamp of the start of Autonomous
    startTime = Timer.getFPGATimestamp();

    // Makes sure gatherer arm is up for the start of the game
    shooter.gathererRetract();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (currentState) {
      case shoot:
        // Actions in case
        drive.stop();
        shooter.intakeStop();
        shooter.setShooterRPM(Shooter.RPM.kHigh);
        if (shooter.isUpToSpeed()) {shooter.indexForwards();} else {shooter.indexStop();}

        // Condition for changing cases
        if (Timer.getFPGATimestamp() - startTime > 1.5 && Timer.getFPGATimestamp() - startTime < 3) {
          currentState = AutoStates.collect;
        }
        break;

      case collect:
        // Actions in case
        drive.pixyAssistedDrive(-.65);
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
        drive.centerOnHub(.65);
        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.intakeStop();
        shooter.indexStop();

        // Condition for changing cases
        if (Math.abs(drive.getRightEncoderDistance()) < 5 || Math.abs(drive.getLeftEncoderDistance()) < 5) {
          timeStamp = Timer.getFPGATimestamp();
          currentState = AutoStates.shoot;
        }
        break;

      case stop:
        // Actions in case
        drive.stop();
        shooter.indexStop();
        shooter.intakeStop();
        shooter.setShooterRPM(Shooter.RPM.kStop);

        // Condition for changing cases
        if (Timer.getFPGATimestamp() - timeStamp > .5) {
          currentState = AutoStates.reverse;
        }
        break;
      
      case aim:
        //Actions
        shooter.indexStop();
        shooter.indexStop();
        shooter.setShooterRPM(Shooter.RPM.kStop);
        drive.centerOnHub(0);

        //Condition for changing
        if(drive.isCenteredOnHub()) {
          currentState = AutoStates.shoot;
        }
      }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Drive control
    if (OI.pixyAssistedDrive()) {
      drive.pixyAssistedDrive(OI.driveThrottle());
    } else if(OI.hubAssistedDrive()){
      drive.centerOnHub(OI.driveThrottle());
    } else {
      drive.XboxDrive();
    }

    // Intake Control
    if(OI.intakeOut()) {
      shooter.intakeOut();
    } else if(OI.intakeIn() || shooter.isUpToSpeed() || OI.pixyAssistedDrive()) {
      shooter.intakeIn();
    } else {
      shooter.intakeStop();
    }

    // Gatherer Control
    if(OI.intakeIn()) {
      shooter.gathererIn();
      shooter.gathererDeploy();
    } else {
      shooter.gathererStop();
      shooter.gathererRetract();
    }


    // Indexer control
    if(OI.moveIndexUp() || shooter.isUpToSpeed()) {
      shooter.indexForwards();
    } else if ((OI.intakeIn() || OI.pixyAssistedDrive()) && shooter.ballIsLoaded() == false) {
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
    if (OI.climbExtend() && !climb.atTop()) {
      climb.unlock();
      if (climbDebouncer.calculate(OI.climbExtend())) {climb.extend();}
    } else if (OI.climbRetract() && !climb.atBottom()) {
      climb.unlock();
      if (climbDebouncer.calculate(OI.climbRetract())) {climb.retract();}
    } else {
      climb.stop();
      climb.lock();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // These likely aren't needed here, but they're here just in case.
    drive.stop();
    shooter.intakeStop();
    shooter.indexStop();
    shooter.setShooterRPM(Shooter.RPM.kStop);
    climb.stop();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
