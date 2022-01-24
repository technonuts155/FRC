package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive {

    // Motor time
    private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.leftMotor1, MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.leftMotor2, MotorType.kBrushless);
    private CANSparkMax leftMotor3 = new CANSparkMax(RobotMap.leftMotor3, MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.rightMotor1, MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.rightMotor2, MotorType.kBrushless);
    private CANSparkMax rightMotor3 = new CANSparkMax(RobotMap.rightMotor3, MotorType.kBrushless);

    // Group left and right motors
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

    // Create drivetrain object
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pixycam
    private Pixy2 pixy;

    public void initializePixy() {
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
    }

    public void pixyShowBlueTargets() {
        int targets = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 10);
        SmartDashboard.putNumber("Blue targets", targets);
        //ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        //Block firstBlock = blocks.get(0);
        SmartDashboard.putBoolean("Red Alliance?", DriverStation.getAlliance() == Alliance.Red); 
    }

    public void invertRightDriveMotors() {
        rightMotors.setInverted(true);
    }

    public void XboxDrive() {

        // Read controller values
        double rightTriggerAxis = OI.driveController.getRightTriggerAxis();
        double leftTriggerAxis = OI.driveController.getLeftTriggerAxis();
        double speed = rightTriggerAxis - leftTriggerAxis;
        double rotation = OI.driveController.getRawAxis(4);

        // Square Inputs, keep values negative if they should be
        if (speed < 0) {
            speed = speed * speed * -1;
        } else {
            speed = speed * speed;
        }

        if (rotation < 0) {
            rotation = rotation * rotation * -1;
        } else {
            rotation = rotation * rotation; 
        }

        // Give values to motor controllers
        drivetrain.arcadeDrive(speed, rotation * .75);
    }

    public void displayMotorControllerInfo() {

        // Displays output currents for each speed controller on dashboard
        SmartDashboard.putNumber("Left Drive 1, Current", leftMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Left Drive 2, Current", leftMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Left Drive 3, Current", leftMotor3.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 1, Current", rightMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 2, Current", rightMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 3, Current", rightMotor3.getOutputCurrent());

        SmartDashboard.putNumber("Left Drive 1, Input", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Input", leftMotor2.get());
        SmartDashboard.putNumber("Left Drive 3, Input", leftMotor3.get());
        SmartDashboard.putNumber("Right Drive 1, Input", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Input", rightMotor2.get());
        SmartDashboard.putNumber("Right Drive 3, Input", rightMotor3.get());
    }

    public void displayDriveControllerAxes() {
        SmartDashboard.putNumber("Left Trigger", OI.driveController.getLeftTriggerAxis());
        SmartDashboard.putNumber("Right Trigger", OI.driveController.getRightTriggerAxis());
        SmartDashboard.putNumber("Right - Left", OI.driveController.getRightTriggerAxis() - OI.driveController.getLeftTriggerAxis());
    }
}
