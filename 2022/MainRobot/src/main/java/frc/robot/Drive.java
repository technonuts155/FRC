package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.ArrayList;
import java.util.List;

public class Drive {
        
    // Motor Controllers and drivetrain
    private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.LEFT_DRIVE_1, MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.LEFT_DRIVE_2, MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.RIGHT_DRIVE_1, MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.RIGHT_DRIVE_2, MotorType.kBrushless);
    
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

    // Create array of motors for easier firmware setting
    private CANSparkMax[] motors = {leftMotor1, leftMotor2, rightMotor1, rightMotor2};

    // Cameras
    private Pixy2 pixy;
    private boolean isCenteredOnHub = false;

    // PIDControllers
    private PIDController pixyPID = new PIDController(0.015, 0.0, 0.001);
    private PIDController encoderPIDLeft = new PIDController(0.008, 0.001, 0.001);
    private PIDController encoderPIDRight = new PIDController(0.008, 0.001, 0.001);
    private PIDController hubPID = new PIDController(0.03, 0.002, 0.0005);

    // Encoders
    private RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;
    private RelativeEncoder rightEncoder1;
    private RelativeEncoder rightEncoder2;
    private double leftZero = 0;
    private double rightZero = 0;

    private double prevInput = 0;
    private double maxInputChange = .035;

    public Drive() {
        // Set firmware settings for motor controllers
        for (CANSparkMax motor : motors) {
            motor.restoreFactoryDefaults();
            motor.setSmartCurrentLimit(70);
            motor.setIdleMode(IdleMode.kBrake);
            motor.burnFlash();
        }

        // Get encoders from motor controllers
        leftEncoder1 = leftMotor1.getEncoder();
        leftEncoder2 = leftMotor2.getEncoder();
        rightEncoder1 = rightMotor1.getEncoder();
        rightEncoder2 = rightMotor2.getEncoder();

        // Initialize Pixy
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();

        // Invert motors on one side
        leftMotors.setInverted(true);
    }

    /** ---------- HUB PID Methods ---------- */

    public void updateHubPIDValues() {
        hubPID.setP(Preferences.getDouble("PID kP", 0.0));
        hubPID.setI(Preferences.getDouble("PID kI", 0.0));
        hubPID.setD(Preferences.getDouble("PID kD", 0.0));
    }

     public void centerOnHub(double speed) {
        if(Vision.hasTargets() == false) {
            drivetrain.arcadeDrive(maxChangeRamp(speed), OI.driveRotation());
            isCenteredOnHub = false;
        } else {
            PhotonTrackedTarget target = Vision.getBestTarget();
            drivetrain.arcadeDrive(maxChangeRamp(speed), hubPID.calculate(target.getYaw()));
            isCenteredOnHub = Math.abs(target.getYaw()) < 4;
        }
     }

     public boolean isCenteredOnHub() {
         return isCenteredOnHub;
     }

    /** ---------- Encoders & Encoder PID Methods ---------- */

    public void encoderPIDDrive() {
            rightMotors.set(encoderPIDRight.calculate(getRightEncoderDistance(), 0));
            leftMotors.set(-encoderPIDLeft.calculate(getLeftEncoderDistance(), 0));
    }

    public void resetEncoders() {
        leftZero = leftEncoder1.getPosition();
        rightZero = rightEncoder1.getPosition();
    }

    public double getLeftEncoderDistance(){
        return leftEncoder1.getPosition() - leftZero;
    }

    public double getRightEncoderDistance(){
        return rightEncoder1.getPosition() - rightZero;
    }

    public void updateEncoderPIDValues() {
        encoderPIDLeft.setP(Preferences.getDouble("PID kP", 0.0));
        encoderPIDLeft.setI(Preferences.getDouble("PID kI", 0.0));
        encoderPIDLeft.setD(Preferences.getDouble("PID kD", 0.0));
        encoderPIDRight.setP(Preferences.getDouble("PID kP", 0.0));
        encoderPIDRight.setI(Preferences.getDouble("PID kI", 0.0));
        encoderPIDRight.setD(Preferences.getDouble("PID kD", 0.0));
    }


    /** ---------- PixyPID Methods ---------- */

    public void pixyAssistedDrive(double speed) {
        Block target = getTargetBlock();
        speed = maxChangeRamp(speed);

        if (target != null) {
            double turnRate = pixyPID.calculate(getBlockCenterX(target), 190);
            drivetrain.arcadeDrive(maxChangeRamp(speed), turnRate);
        } else {
            drivetrain.arcadeDrive(maxChangeRamp(speed), OI.driveRotation());
        }
    }

    public void resetPixyPID() {
        pixyPID.reset();
    }

    public void setPixyPIDSetpoint(double setpoint) {
        pixyPID.setSetpoint(setpoint);
    }

    public void setPixyPIDTolerance(double tolerance) {
        pixyPID.setTolerance(tolerance);
    }

    public void updatePixyPIDValues() {
        pixyPID.setP(Preferences.getDouble("PID kP", 0.0));
        pixyPID.setI(Preferences.getDouble("PID kI", 0.0));
        pixyPID.setD(Preferences.getDouble("PID kD", 0.0));
    }


    /** ---------- Pixy Vision Processing Methods ---------- */

    private double getBlockCenterX(Block block) {
        return (block.getX() + (block.getWidth()/2));
    }

    public double getAreaOfBlock(Block block) {
        return (block.getHeight() * block.getWidth());
    }

    public boolean matchesBlockRatioHW(Block block, double tolerance) {
        double ratio = block.getWidth() / block.getHeight();
        return (ratio <= 1 + tolerance && ratio >= 1 - tolerance);
    }

    public Block getLargestBlock(ArrayList<Block> blocks) {
        Block largestBlock = null;
        for(Block block : blocks) {
            if(largestBlock == null) {
                largestBlock = block; 
            } else {
                if(getAreaOfBlock(largestBlock) < getAreaOfBlock(block)) {
                    largestBlock = block;
                }
            }  
        }
        return largestBlock;
    }

    public Block getTargetBlock() {

        // Get blocks based on alliance color
        /**
        if (DriverStation.getAlliance() == Alliance.Blue) {
            pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG1, 10);
        } else {
            pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG2, 10);
        }
        */

        // Getting red AND blue during drive practice
        pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG1, 10);
        pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG2, 10);

        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

        // Create new list of only blocks that are within Height/Width ratio tolerance
        ArrayList<Block> ratioedBlocks = new ArrayList<Block>();
        for(Block block : blocks) {
            if(matchesBlockRatioHW(block, .5) == true) {
                ratioedBlocks.add(block);
            }
        }

        // Return the area-wise largest block of the remaining blocks
        return getLargestBlock(ratioedBlocks);
    }


    /** ---------- Manual Drive Methods ---------- */

    public void XboxDrive() {
        drivetrain.arcadeDrive(maxChangeRamp(OI.driveThrottle()), OI.driveRotation());
        isCenteredOnHub = false;
    }

    public void stop() {
        drivetrain.arcadeDrive(maxChangeRamp(0), 0);
    }
    
    private double maxChangeRamp(double input) {
        if (input < prevInput) {
            if (input < prevInput - maxInputChange) {
                prevInput -= maxInputChange;
            } else {
                prevInput = input;
            }
        } else {
            if (input > prevInput + maxInputChange) {
                prevInput += maxInputChange;
            } else {
                prevInput = input;
            }
        }

        return prevInput;
    }

    public void displayMotorControllerInputs() {
        // Displays the input given to each speed controller (range of -1 -> 1)
        SmartDashboard.putNumber("Left Drive 1, Input", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Input", leftMotor2.get());
        SmartDashboard.putNumber("Right Drive 1, Input", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Input", rightMotor2.get());
    }
}