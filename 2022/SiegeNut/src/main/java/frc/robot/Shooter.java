package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter {
    public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(RobotMap.intakeMotor);
    public WPI_TalonSRX shooterMotor = new WPI_TalonSRX(RobotMap.shooterMotor);
    public DigitalInput beamBreak = new DigitalInput(RobotMap.beamBreak);

    public void setShootGo() {
        shooterMotor.set(ControlMode.PercentOutput, OI.shooterOutput());
    }

    public void intakeIn() {
        intakeMotor.set(0.75);
    }
    public void intakeOut() {
        intakeMotor.set(-0.75);
    }
    public void intakeStop() {
        intakeMotor.set(0);
    }
    public boolean getBeamBreak() {
        return beamBreak.get();
    }
}

