package frc.robot;


import java.util.Dictionary;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Shooter {

    // Control states for the shooter flywheel
    enum RPM {          // Black magic voodoo enum
        kStatic,        // Aim from closer (top goal)
        kDynamic,       // Aim from farther away (top goal)
        kStop           // Stop the motor
    }
    private double[] rpmTable = {0, 0, 0, 3700, 3700, 3900, 4100, 4450, 4600, 4900};

    // Motor Controllers
    private CANSparkMax shooterMotor = new CANSparkMax(RobotMap.SHOOTER_MOTOR, MotorType.kBrushless);
    private VictorSPX indexerMotor = new VictorSPX(RobotMap.INDEXER_MOTOR);
    private VictorSPX intakeMotor = new VictorSPX(RobotMap.INTAKE_MOTOR);
    private VictorSPX gathererMotor = new VictorSPX(RobotMap.GATHERER_MOTOR);

    // Solenoids
    private DoubleSolenoid solenoidA = new DoubleSolenoid(RobotMap.PCM, PneumaticsModuleType.CTREPCM, RobotMap.SOL1_SLOT_A, RobotMap.SOL1_SLOT_B);
    private DoubleSolenoid solenoidB = new DoubleSolenoid(RobotMap.PCM, PneumaticsModuleType.CTREPCM, RobotMap.SOL2_SLOT_A, RobotMap.SOL2_SLOT_B); 

    // PID Controller and Encoder for controlling the shooter motor
    private SparkMaxPIDController pid;
    private RelativeEncoder shooterEncoder;

    // RPM setpoint constants for shooter control
<<<<<<< Updated upstream
    private final double HIGH = 3900.0;
=======
    private final double HIGH = 3700.0;

    // Color sensor and Beam Breaks
    DigitalInput beamBreakHigh = new DigitalInput(RobotMap.BEAM_BREAK_HIGH);
    DigitalInput beamBreakLow = new DigitalInput(RobotMap.BEAM_BREAK_LOW);

    //Delay for shooter
    private double shooterDelay = .05;
    private double shooterTimestamp = 0;

    private boolean highBeamBreakWasTrue = getBeamBreakHigh();

>>>>>>> Stashed changes

    // Tolerance for shooter RPM and boolean for being within tolerance (shooter is up to speed)
    private double tolerance = 100;   
    private boolean upToSpeed = false;

    // Color sensor and Beam Breaks
    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    DigitalInput beamBreakHigh = new DigitalInput(RobotMap.BEAM_BREAK_HIGH);
    DigitalInput beamBreakLow = new DigitalInput(RobotMap.BEAM_BREAK_LOW);
    
    public Shooter() {

        shooterEncoder = shooterMotor.getEncoder();

        pid = shooterMotor.getPIDController();
        pid.setP(0.0003);
        pid.setI(0);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0.00019);
        pid.setOutputRange(-1, 1);
        pid.setFeedbackDevice(shooterEncoder);

        gathererRetract();
    }


    /** ---------- Shooter Methods ---------- */

    public void setShooterRPM(Shooter.RPM rpm) {
        // Set RPM reference and update upToSpeed boolean
        switch (rpm) {
            case kStatic:
                pid.setReference(HIGH, ControlType.kVelocity);
                upToSpeed = Math.abs(HIGH - shooterEncoder.getVelocity()) <= tolerance || shooterEncoder.getVelocity() > HIGH;
                break;

            case kDynamic:
                if (Vision.hasTargets() && Vision.distanceFromHub() > 3 && Vision.distanceFromHub() < 8) {
                    double distance = Vision.distanceFromHub();
                    double RPM = rpmTable[(int)distance] + ((rpmTable[(int)distance + 1] - rpmTable[(int)distance]) * (distance % 1));
                    pid.setReference(RPM, ControlType.kVelocity);
                    upToSpeed = Math.abs(RPM - shooterEncoder.getVelocity()) <= tolerance || shooterEncoder.getVelocity() > RPM;
                } else {
                    shooterMotor.set(0);
                    upToSpeed = false;
                }
                break;

            case kStop:
                shooterMotor.set(0);
                upToSpeed = false;
                break;
        }
    }

    public double getTemperature() {
        return shooterMotor.getMotorTemperature();
    }

    public void setShooterPercentOutput(double output) {
        shooterMotor.set(output);
    }

    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public boolean isUpToSpeed() {
<<<<<<< Updated upstream
        return upToSpeed;
=======
        if (highBeamBreakWasTrue && getBeamBreakHigh() == false) {
            shooterTimestamp = Timer.getFPGATimestamp();
        }
        highBeamBreakWasTrue = getBeamBreakHigh();

        if(upToSpeed == true && Timer.getFPGATimestamp() - shooterTimestamp >= shooterDelay) 
            return true;
        else
            return false;
>>>>>>> Stashed changes
    }

    public void updatePIDValues() {
        pid.setP(Preferences.getDouble("PID kP", 0.0));
        pid.setI(Preferences.getDouble("PID kI", 0.0));
        pid.setD(Preferences.getDouble("PID kD", 0.0));
        pid.setFF(Preferences.getDouble("PID FF", 0.0));
        tolerance = Preferences.getDouble("Tolerance", 100);
    }


    /** --------- Gatherer Methods --------- */

    public void gathererIn() {
        gathererMotor.set(ControlMode.PercentOutput, 1);
    }
    public void gathererOut() {
        gathererMotor.set(ControlMode.PercentOutput, -.5);
    }
    public void gathererStop() {
        gathererMotor.set(ControlMode.PercentOutput, 0);
    }
    public void gathererDeploy() {
        solenoidA.set(Value.kForward);
        solenoidB.set(Value.kForward);
    }
    public void gathererRetract() {
        solenoidA.set(Value.kReverse);
        solenoidB.set(Value.kReverse);
    }



    /** ---------- Intake Methods ---------- */

    public double getIntakeSpeed() {
        return intakeMotor.getMotorOutputPercent();
    }
    public void intakeOut() {
        intakeMotor.set(ControlMode.PercentOutput, 1);
    }

    public void intakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, -1);
    }

    public void intakeStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }


    /** ---------- Indexer Methods ---------- */

    public double getIndexSpeed() {
        return indexerMotor.getMotorOutputPercent();
    }

    /** Moves the belt up slow enough for the color sensor to recognize a ball */
    public void indexForwardsSlow() {
        indexerMotor.set(ControlMode.PercentOutput, .45);
    }

    public void indexForwards() {
        indexerMotor.set(ControlMode.PercentOutput, .75);
    }

    public void indexBackwards() {
        indexerMotor.set(ControlMode.PercentOutput, -.5);
    }
   
    public void indexStop() {
        indexerMotor.set(ControlMode.PercentOutput, 0);
    }


    /** ---------- Beam Break Methods ---------- */

    public boolean getBeamBreakLow() {
        return !beamBreakLow.get();
    }

    public boolean getBeamBreakHigh() {
        return !beamBreakHigh.get();
    }

    public boolean ballIsLoaded() {
        return getBeamBreakLow() || getBeamBreakHigh();
    }
}