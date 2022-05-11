package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Drive {

    private static WPI_TalonSRX leftDrive1 = new WPI_TalonSRX(RobotMap.leftDriveMotor1);
    private static WPI_TalonSRX leftDrive2 = new WPI_TalonSRX(RobotMap.leftDriveMotor2);
    private static WPI_TalonSRX rightDrive1 = new WPI_TalonSRX(RobotMap.rightDriveMotor1);
    private static WPI_TalonSRX rightDrive2 = new WPI_TalonSRX(RobotMap.rightDriveMotor2);

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
