// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private WPI_TalonSRX intake = new WPI_TalonSRX(2);

  double target_height = Units.inchesToMeters(104);
  double camera_height = Units.inchesToMeters(37);
  double camera_pitch = Units.degreesToRadians(50);

  //Drive drive = new Drive();
  
  //DoubleSolenoid solenoidA = new DoubleSolenoid(12, PneumaticsModuleType.CTREPCM, RobotMap.solenoidA_1, RobotMap.solenoidA_2);
  //DoubleSolenoid solenoidB = new DoubleSolenoid(12, PneumaticsModuleType.CTREPCM, RobotMap.solenoidB_1, RobotMap.solenoidB_2);

  PhotonCamera hubCam = new PhotonCamera("HUB Cam");
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    

    //solenoidA.set(Value.kReverse);
    //solenoidB.set(Value.kReverse);
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
    PhotonPipelineResult result = hubCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("Has Targets", hasTargets);

    if (hasTargets) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      SmartDashboard.putNumber("# Targets", targets.size());

      PhotonTrackedTarget bestTarget = result.getBestTarget();
      
      double pitch = bestTarget.getPitch();
      double yaw = bestTarget.getYaw();
      double area = bestTarget.getArea();

      double distance = PhotonUtils.calculateDistanceToTargetMeters(camera_height,
                                                                    target_height,
                                                                    camera_pitch,
                                                                    Units.degreesToRadians(pitch));

      


      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Distance", Units.metersToFeet(distance));
    }
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
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /*
    drive.XboxDrive();

    if (OI.driveController.getXButtonPressed()) {
      solenoidA.toggle();
    }
    if (OI.driveController.getBButtonPressed()) {
      solenoidB.toggle();
    }
    */

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
  public void testPeriodic() {}


}
