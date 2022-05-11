package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter {
    private static WPI_TalonSRX intakeMotor = new WPI_TalonSRX(RobotMap.intakeMotor);
    private static WPI_TalonSRX shooterMotor = new WPI_TalonSRX(RobotMap.shooterMotor);
    private static DigitalInput beamBreak = new DigitalInput(RobotMap.beamBreak);

    public static void setShootGo() {
        shooterMotor.set(ControlMode.PercentOutput, OI.shooterOutput());
    }

    public static void intakeIn() {
        intakeMotor.set(0.75);
    }
    public static void intakeOut() {
        intakeMotor.set(-0.75);
    }
    public static void intakeStop() {
        intakeMotor.set(0);
    }
    public static boolean getBeamBreak() {
        return beamBreak.get();
    }
}

