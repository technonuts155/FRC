package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Drive {

    private static PWMTalonSRX leftDrive1 = new PWMTalonSRX(RobotMap.leftDriveMotor1);
    private static PWMTalonSRX leftDrive2 = new PWMTalonSRX(RobotMap.leftDriveMotor2);
    private static PWMTalonSRX rightDrive1 = new PWMTalonSRX(RobotMap.rightDriveMotor1);
    private static PWMTalonSRX rightDrive2 = new PWMTalonSRX(RobotMap.rightDriveMotor2);

    private static MotorControllerGroup leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
    private static MotorControllerGroup rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);

    private static DifferentialDrive drivetrain = new DifferentialDrive(leftDrive, rightDrive);

    public static void invertLeftMotors() {
        leftDrive.setInverted(true);
    }

    public static void arcadeDrive() {
        drivetrain.arcadeDrive(OI.getDriveThrottle(), OI.getDriveRotation());
    }
}