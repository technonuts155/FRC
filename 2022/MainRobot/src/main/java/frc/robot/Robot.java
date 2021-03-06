// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  Debouncer climbDebouncer = new Debouncer(.5, DebounceType.kBoth);

  // Delay for second ball when shooting (in seconds)
  double shootDelay = .5;
  double shootTimestamp = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    climb.unlock();
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
      // Vision.updatePipelineResult() needs to be called here or vision tracking will fail
      Vision.updatePipelineResult();
      SmartDashboard.putNumber("Distance", Vision.distanceFromHub());
      SmartDashboard.putBoolean("Has Targets", Vision.hasTargets());
      SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
      SmartDashboard.putBoolean("Low Climb", climb.atBottom());
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
    currentState = AutoStates.collect;

    // Zero the encoders
    drive.resetEncoders();

    // Get a system timestamp of the start of Autonomous
    startTime = Timer.getFPGATimestamp();
    timeStamp = Timer.getFPGATimestamp();

    // Makes sure gatherer arm is up for the start of the game
    shooter.gathererRetract();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putString("Auto State", currentState.toString());
    SmartDashboard.putBoolean("Beam break high", shooter.getBeamBreakHigh());
    SmartDashboard.putBoolean("Beam Break low", shooter.getBeamBreakLow());
    SmartDashboard.putBoolean("Ball is loaded", shooter.ballIsLoaded());
    switch (currentState) {
      case shoot:
        // Actions in case
        drive.stop();
        shooter.intakeStop();
        shooter.gathererRetract();
        shooter.gathererStop();

        if (Timer.getFPGATimestamp() - timeStamp < 3) {
          shooter.setShooterRPM(Shooter.RPM.kStatic);
        } else {
          shooter.setShooterRPM(Shooter.RPM.kStop);
        }
        
        if (shooter.isUpToSpeed()) {
          shooter.indexForwards();
        } else {
          shooter.indexStop();
        }

        // Condition for changing cases
        if (Timer.getFPGATimestamp() - startTime > 1.5 && Timer.getFPGATimestamp() - startTime < 3) {
          currentState = AutoStates.collect;
        }
        break;

      case collect:
        // Actions in case
        drive.pixyAssistedDrive(-.5);
        shooter.intakeIn();
        shooter.indexForwardsSlow();
        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.gathererDeploy();
        shooter.gathererIn();

        // Condition for changing cases
        if(shooter.ballIsLoaded() == true || Timer.getFPGATimestamp() - timeStamp > 2.5) {
          timeStamp = Timer.getFPGATimestamp();
          currentState = AutoStates.stop;
        }
        break;
      
      case reverse:
        // Actions in case
        drive.centerOnHub(.5);
        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.intakeStop();
        shooter.indexStop();
        shooter.gathererRetract();
        shooter.gathererStop();

        // Condition for changing cases
        if (Math.abs(drive.getRightEncoderDistance()) < 10 || Math.abs(drive.getLeftEncoderDistance()) < 10) {
          timeStamp = Timer.getFPGATimestamp();
          currentState = AutoStates.shoot;
        }
        break;

      case stop:
        // Actions in case
        drive.stop();
        shooter.setShooterRPM(Shooter.RPM.kStop);
        shooter.gathererRetract();
        shooter.gathererStop();

        if (shooter.getBeamBreakHigh()) {
          shooter.indexStop();
          shooter.intakeStop();
        } else {
          shooter.indexForwardsSlow();
          shooter.intakeIn();
        }

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
  public void teleopInit() {
    climb.unlock();
  }

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
    if (shooter.isUpToSpeed() == false) {
      shootTimestamp = Timer.getFPGATimestamp();
    }

    if(OI.moveIndexUp() || (shooter.isUpToSpeed() && Timer.getFPGATimestamp() - shootTimestamp > shootDelay)) {
      shooter.indexForwards();
    } else if ((OI.intakeIn() || OI.pixyAssistedDrive()) && shooter.getBeamBreakHigh() == false) {
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
      if (OI.shootStaticRPM()) {
        shooter.setShooterRPM(Shooter.RPM.kStatic); 
      } else if (OI.shootDynamicRPM()) {
        shooter.setShooterRPM(Shooter.RPM.kDynamic);
      } else {
        shooter.setShooterRPM(Shooter.RPM.kStop);;
      }
    }

    // Climber control
    if (OI.climbExtend() && !climb.atTop()) {
        climb.extend();
    } else if (OI.climbRetract() && !climb.atBottom()) {
        climb.retract();
    } else {
        climb.stop();
    }

    if (OI.climbLock()) {
      climb.lock();
    }

    if (OI.climbUnlock()) {
      climb.unlock();
    }

    // Rumble City
    if ((drive.isCenteredOnHub() && Vision.distanceFromHub() >= 3 && Vision.distanceFromHub() <= 6.5) ||
       ((OI.pixyAssistedDrive() || OI.intakeIn()) && shooter.getBeamBreakLow())) {
      OI.driverController.setRumble(RumbleType.kLeftRumble, .75);
      OI.driverController.setRumble(RumbleType.kRightRumble, .75);
      OI.operatorController.setRumble(RumbleType.kLeftRumble, .75);
      OI.operatorController.setRumble(RumbleType.kRightRumble, .75);
    } else {
      OI.driverController.setRumble(RumbleType.kLeftRumble, 0);
      OI.driverController.setRumble(RumbleType.kRightRumble, 0);
      OI.operatorController.setRumble(RumbleType.kLeftRumble, 0);
      OI.operatorController.setRumble(RumbleType.kRightRumble, 0);
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
