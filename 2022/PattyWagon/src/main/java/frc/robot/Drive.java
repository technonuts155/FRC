package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    // Variables
    private double speedLimit = 1;
    public void invertRightDriveMotors() {
        rightMotors.setInverted(true);
    }

    private void speedLimitControl() {
        if(OI.driveController.getAButtonPressed() == true && speedLimit > 0.1) {
            speedLimit = speedLimit - 0.1;
        }
        if(OI.driveController.getYButtonPressed() == true && speedLimit < 1) {
            speedLimit = speedLimit + 0.1;
        }
        SmartDashboard.putNumber("speed limit control #", speedLimit);
    }

    public void XboxDrive() {

        speedLimitControl();

        // Read controller values
        double rightTriggerAxis = OI.driveController.getRightTriggerAxis();
        double leftTriggerAxis = OI.driveController.getLeftTriggerAxis();
        double speed = OI.driveController.getRawAxis(1) * -1;
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
        drivetrain.arcadeDrive(speed * speedLimit, rotation * .50 * speedLimit);
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
