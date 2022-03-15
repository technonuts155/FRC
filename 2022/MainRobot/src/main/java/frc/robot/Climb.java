package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class Climb {

    // Motors
    private VictorSPX climbMotor1 = new VictorSPX(RobotMap.CLIMB_MOTOR_1);
    private VictorSPX climbMotor2 = new VictorSPX(RobotMap.CLIMB_MOTOR_2);

    // Limit Switches
    private DigitalInput lower = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_LOW);
    private DigitalInput upper = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_UPPER);

    // Locking servo and state boolean
    private Servo climbLock = new Servo(RobotMap.CLIMB_SERVO);
    private boolean isLocked = false;

    public Climb() {
        lock();
    }

    public void lock() {
        climbLock.setAngle(30);
        isLocked = true;
    }

    public void unlock() {
        climbLock.setAngle(0);
        isLocked = false;
    }

    public boolean isLocked() {
        return isLocked;
    }

    public boolean atBottom() {
        return lower.get();
    }
    public boolean atTop() {
        return upper.get();
    }

    public void extend() {
        climbMotor1.set(ControlMode.PercentOutput, -.75);
        climbMotor2.set(ControlMode.PercentOutput, -.75);
    }

    public void retract() {
        climbMotor1.set(ControlMode.PercentOutput, .75);
        climbMotor2.set(ControlMode.PercentOutput, .75);
    }
    public void stop() {
        climbMotor1.set(ControlMode.PercentOutput, 0.0);
        climbMotor2.set(ControlMode.PercentOutput, 0.0);
    }

    public void setClimbPercentOutput(double speed) {
        climbMotor1.set(ControlMode.PercentOutput, speed);
        climbMotor2.set(ControlMode.PercentOutput, speed);
    }
}
