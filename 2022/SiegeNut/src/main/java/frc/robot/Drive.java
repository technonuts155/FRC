package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Drive {

    WPI_TalonSRX leftDrive1 = new WPI_TalonSRX(RobotMap.leftDriveMotor1);
    WPI_TalonSRX leftDrive2 = new WPI_TalonSRX(RobotMap.leftDriveMotor2);
    WPI_TalonSRX rightDrive1 = new WPI_TalonSRX(RobotMap.rightDriveMotor1);
    WPI_TalonSRX rightDrive2 = new WPI_TalonSRX(RobotMap.rightDriveMotor2);

    MotorControllerGroup leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
    MotorControllerGroup rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);

    DifferentialDrive drivetrain = new DifferentialDrive(leftDrive, rightDrive);

    public Drive() {
        leftDrive.setInverted(true);
    }

    public void arcadeDrive() {
        drivetrain.arcadeDrive(OI.getDriveThrottle(), OI.getDriveRotation());
    }

}
