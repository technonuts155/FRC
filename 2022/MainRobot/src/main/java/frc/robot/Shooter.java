package frc.robot;


import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DigitalInput;


public class Shooter {

    // Control states for the shooter flywheel
    enum RPM { //black magic voodoo enum
        kHigh,      // Aim for high goal
        kLow,       // Aim for low goal
        kStop       // Stop motor
    }

    // Motor Controllers
    private CANSparkMax shooterMotor = new CANSparkMax(RobotMap.SHOOTER_MOTOR, MotorType.kBrushless);
    private TalonSRX indexerMotor = new TalonSRX(RobotMap.INDEXER_MOTOR);
    private TalonSRX intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);

    // PID Controller and Encoder for controlling the shooter motor
    private SparkMaxPIDController pid;
    private RelativeEncoder shooterEncoder;

    // RPM setpoint constants for shooter control
    private final double HIGH = 300.0;
    private final double LOW = 100;
    private final double STOP = 0;

    // Tolerance for shooter RPM and boolean for being within tolerance (shooter is up to speed)
    private final double tolerance = 100;   
    private boolean upToSpeed = false;
    
    //Beam Break
    DigitalInput beamBreak = new DigitalInput(RobotMap.BEAM_BREAK);

    // PID functions
    public void initPID() {
        pid = shooterMotor.getPIDController();
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0);
        pid.setOutputRange(-1, 1);
        pid.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public void updatePIDValues() {
        pid.setP(Preferences.getDouble("pid kP", 0.0));
        pid.setI(Preferences.getDouble("pid kI", 0.0));
        pid.setD(Preferences.getDouble("pid kD", 0.0));
        pid.setIZone(Preferences.getDouble("pid IZone", 0.0));
        pid.setFF(Preferences.getDouble("pid FF", 0.0));
    }
    
    public void initalizeEncoder() {
        shooterEncoder = shooterMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    }

    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public boolean isUpToSpeed() {
        return upToSpeed;
    }
    
    public void setShooterRPM(Shooter.RPM rpm) {

        // Set RPM reference and update upToSpeed boolean
        switch (rpm) {
            case kHigh:
                pid.setReference(HIGH, ControlType.kVelocity);
                upToSpeed = Math.abs(HIGH - shooterEncoder.getVelocity()) <= tolerance;
                break;

            case kLow:
                pid.setReference(LOW, ControlType.kVelocity);
                upToSpeed = Math.abs(LOW - shooterEncoder.getVelocity()) <= tolerance;
                break;

            case kStop:
                pid.setReference(STOP, ControlType.kVelocity);
                upToSpeed = false;
                break;
        }
    } 

    //Intake Functions
    public double getIntakeSpeed() {
        return intakeMotor.getMotorOutputPercent();
    }
    public void intakeForwards() {
        intakeMotor.set(ControlMode.PercentOutput, 1);
    }

    public void intakeReverse() {
        intakeMotor.set(ControlMode.PercentOutput, -1);
    }

    public void intakeStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    // Index functions 
    public double getIndexSpeed() {
        return indexerMotor.getMotorOutputPercent();
    }

    public void indexForwards() {
        indexerMotor.set(ControlMode.PercentOutput, 1);
    }

    public void indexBackwards() {
        indexerMotor.set(ControlMode.PercentOutput, -1);
    }
   
    public void indexStop() {
        indexerMotor.set(ControlMode.PercentOutput, 0);
    }
    
    // Beambreak
    public boolean getBeamBreak() {
        return beamBreak.get();
    }
}