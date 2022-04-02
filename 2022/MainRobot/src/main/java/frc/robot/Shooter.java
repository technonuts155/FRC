package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Shooter {

    // Control states for the shooter flywheel
    enum RPM {      // Black magic voodoo enum
        kHigh,      // Aim for high goal
        kLow,       // Aim for low goal
        kStop       // Stop motor
    }

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
    private final double HIGH = 3700.0;
    private final double LOW = 2700.0;

    // Tolerance for shooter RPM and boolean for being within tolerance (shooter is up to speed)
    private final double tolerance = 100;   
    private boolean upToSpeed = false;

    // Color sensor
    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    
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
            case kHigh:
                pid.setReference(HIGH, ControlType.kVelocity);
                upToSpeed = Math.abs(HIGH - shooterEncoder.getVelocity()) <= tolerance;
                break;

            case kLow:
                pid.setReference(LOW, ControlType.kVelocity);
                upToSpeed = Math.abs(LOW - shooterEncoder.getVelocity()) <= tolerance;
                break;

            case kStop:
                shooterMotor.set(0);
                upToSpeed = false;
                break;

            default:
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
        return upToSpeed;
    }

    public void updatePIDValues() {
        pid.setP(Preferences.getDouble("PID kP", 0.0));
        pid.setI(Preferences.getDouble("PID kI", 0.0));
        pid.setD(Preferences.getDouble("PID kD", 0.0));
        pid.setFF(Preferences.getDouble("PID FF", 0.0));
    }

    /** --------- Gatherer Methods --------- */
    public void gathererIn() {
        gathererMotor.set(ControlMode.PercentOutput, -.5);
    }
    public void gathererOut() {
        gathererMotor.set(ControlMode.PercentOutput, .5);
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
        intakeMotor.set(ControlMode.PercentOutput, 0.75);
    }

    public void intakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, -0.75);
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
        indexerMotor.set(ControlMode.PercentOutput, .4);
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


    /** ---------- Color Sensor Methods ---------- */

    public boolean ballIsLoaded() {
        return (colorSensor.getIR() > 8);
    }

    public int getColorSensorIR() {
        return colorSensor.getIR();
    }
}