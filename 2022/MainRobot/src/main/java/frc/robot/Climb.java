package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Climb {
    private VictorSPX climbMotor1 = new VictorSPX(RobotMap.CLIMB_MOTOR_1);
    private VictorSPX climbMotor2 = new VictorSPX(RobotMap.CLIMB_MOTOR_2);
    private DigitalInput lower = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_LOW);
    private DigitalInput upper = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_UPPER);
    private DigitalInput middle = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_MIDDLE);

    public boolean getLimitLower() {
        return lower.get();
    }
    public boolean getLimitUpper() {
        return upper.get();
    }
    public boolean getLimitMiddle() {
        return middle.get();
    }
    public void setClimbUp() {
        climbMotor1.set(ControlMode.PercentOutput, 1);
        climbMotor2.set(ControlMode.PercentOutput, 1);
    }
    public void setClimbDown() {
        climbMotor1.set(ControlMode.PercentOutput, -1);
        climbMotor2.set(ControlMode.PercentOutput, -1);
    }
    public void setClimbStop() {
        climbMotor1.set(ControlMode.PercentOutput, 0.0);
        climbMotor2.set(ControlMode.PercentOutput, 0.0);
    }

    public void setClimbPercentOutput(double speed) {
        climbMotor1.set(ControlMode.PercentOutput, speed);
        climbMotor2.set(ControlMode.PercentOutput, speed);
    }
}
