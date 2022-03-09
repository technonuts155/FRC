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



import java.util.ArrayList;

public class Drive {
    // Motor time
    
    private WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(RobotMap.LEFT_DRIVE_1);
    private WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(RobotMap.LEFT_DRIVE_2);
    private WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(RobotMap.RIGHT_DRIVE_1);
    private WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(RobotMap.RIGHT_DRIVE_2);

    // Group left and right motors
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

    // Create drivetrain object
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pixycam
    private Pixy2 pixy;
    private final double HORIZONTAL_CENTER = 157.5;

    // PIDController for centering on target found by pixycam
    private PIDController pixyPID = new PIDController(0.015, 0.0, 0.001);
    private PIDController encoderPIDLeft = new PIDController(0, 0, 0);
    private PIDController encoderPIDRight = new PIDController(0, 0, 0);

    // Encoders
    private Encoder leftDriveEncoder = new Encoder(RobotMap.LEFT_DRIVE_ENCODER_A, RobotMap.LEFT_DRIVE_ENCODER_B);
    private Encoder rightDriveEncoder = new Encoder(RobotMap.RIGHT_DRIVE_ENCODER_A, RobotMap.RIGHT_DRIVE_ENCODER_B);
    private final double PULSES_TO_INCHES = 1 / 18.9231;
    private final double PULSES_TO_FEET = PULSES_TO_INCHES * 12;

    public void updateEncoderPIDValues() {
        encoderPIDLeft.setP(Preferences.getDouble("PID kP", 0.0));
        encoderPIDLeft.setI(Preferences.getDouble("PID kI", 0.0));
        encoderPIDLeft.setD(Preferences.getDouble("PID kD", 0.0));
        encoderPIDRight.setP(Preferences.getDouble("PID kP", 0.0));
        encoderPIDRight.setI(Preferences.getDouble("PID kI", 0.0));
        encoderPIDRight.setD(Preferences.getDouble("PID kD", 0.0));
    }

    public void encoderPIDDrive() {
        if (Math.abs(getRightEncoderDistance()) > 5) {
            rightMotors.set(encoderPIDRight.calculate(getRightEncoderDistance(), 0));
        } else {
            rightMotors.set(0);
        }

        if (Math.abs(getLeftEncoderDistance()) > 5) {
            leftMotors.set(-encoderPIDLeft.calculate(getLeftEncoderDistance(), 0));
        } else {
            leftMotors.set(0);
        }
    }

    public void resetEncoders() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
    }

    public void initializeEncoders(){
        leftDriveEncoder.setDistancePerPulse(PULSES_TO_INCHES);
        rightDriveEncoder.setDistancePerPulse(PULSES_TO_INCHES);
    }

    public double getLeftEncoderDistance(){
        return leftDriveEncoder.getDistance();
    }

    public double getRightEncoderDistance(){
        return rightDriveEncoder.getDistance();
    }
    
    public void resetPID() {
        pixyPID.reset();
    }

    // Used for tuning the PID
    // Pulls numbers from the Preferences box of the Smartdashboard
    public void updatePIDValues() {
        pixyPID.setP(Preferences.getDouble("pixyPID kP", 0.0));
        pixyPID.setI(Preferences.getDouble("pixyPID kI", 0.0));
        pixyPID.setD(Preferences.getDouble("pixyPID kD", 0.0));
        pixyPID.setTolerance(Preferences.getDouble("pixyPID Tolerance", 10));
        pixyPID.setSetpoint(Preferences.getDouble("pixyPID Setpoint", 157.5));
    }

    

    public void setPIDSetpoint(double setpoint) {
        pixyPID.setSetpoint(setpoint);
    }

    public void setPIDTolerance(double tolerance) {
        pixyPID.setTolerance(tolerance);
    }

    private double getBlockCenterX(Block block) {
        return (block.getX() + (block.getWidth()/2));
    }

    public double getAreaOfBlock(Block block) {
        return (block.getHeight() * block.getWidth());
    }

    public boolean matchesAllianceColor(Block block) {
        if(DriverStation.getAlliance() == Alliance.Blue) {
            //it gon' be blue
            return(block.getSignature() == 1);
        } else {
            //it gon' be red
            return (block.getSignature() == 2);
        }
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

        // Getting block cache doesn't work if you don't get block count first
        int blocksFound;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            blocksFound = pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG1, 10);
        } else {
            blocksFound = pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG2, 10);
        }
        SmartDashboard.putNumber("Blocks Found", blocksFound);

        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        ArrayList<Block> colorBlocks = new ArrayList<Block>();
        ArrayList<Block> ratioedBlocks = new ArrayList<Block>();

        // Create new list of only blocks matching our alliance color
        for(Block block : blocks) {
            if(matchesAllianceColor(block) == true) {
                colorBlocks.add(block);
            }
        }

        // Create new list of only blocks that are square enough
        // Perfectly identified cargo should be a square because
        // a bounding box drawn around a circle is a square.
        for(Block block : colorBlocks) {
            if(matchesBlockRatioHW(block, .5) == true) {
                ratioedBlocks.add(block);
            }
        }

        SmartDashboard.putNumber("All blocks", blocks.size());
        SmartDashboard.putNumber("Color Blocks", colorBlocks.size());
        SmartDashboard.putNumber("Ratio Blocks", ratioedBlocks.size());

        // Return the area-wise largest block of the remaining blocks
        return getLargestBlock(ratioedBlocks);
    }

    public void initializePixy() {
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
    }

    public void invertRightDriveMotors() {
        rightMotors.setInverted(true);
    }

    public void invertLeftDriveMotors() {
        leftMotors.setInverted(true);
    }

    public void pixyAutopilot(double speed) {
        Block target = getTargetBlock();

        if (target != null) {
            double turnRate = pixyPID.calculate(getBlockCenterX(target), 190);
            drivetrain.arcadeDrive(speed, turnRate);
        } else {
            drivetrain.arcadeDrive(speed, OI.driveRotation() * -1);
        }
    }

    public void PIDAtSetpoint() {
        SmartDashboard.putBoolean("At setpoint", pixyPID.atSetpoint());
    }

    public void displayPIDValues() {
        SmartDashboard.putNumber("P", pixyPID.getP());
        SmartDashboard.putNumber("I", pixyPID.getI());
        SmartDashboard.putNumber("D", pixyPID.getD());
    }

    public void setLeftMotors(double speed) {
        leftMotors.set(speed);
    }

    public void setRightMotors(double speed) {
        rightMotors.set(speed);
    }

    public void XboxDrive() {

        // Read controller values
        double speed = OI.driveThrottle();
        double rotation = OI.driveRotation();

        // Square Inputs, keep values negative if they should be
        if (speed < 0) {
            speed = speed * speed * -1;
        } else {
            speed = speed * speed;
        }

        if (rotation < 0) {
            rotation = rotation * rotation * -1;
        } else {
            rotation = rotation * rotation; 
        }

        // Give values to motor controllers
        drivetrain.arcadeDrive(speed, rotation * -1);
    }

    public void displayMotorControllerOutputCurrents() {
        // Displays output currents for each speed controller (in amps)
        SmartDashboard.putNumber("Left Drive 1, Current", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Current", leftMotor2.get());
        SmartDashboard.putNumber("Right Drive 1, Current", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Current", rightMotor2.get());
    }

    public void displayMotorControllerInputs() {
        // Displays the input given to each speed controller (range of -1 -> 1)
        SmartDashboard.putNumber("Left Drive 1, Input", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Input", leftMotor2.get());
        SmartDashboard.putNumber("Right Drive 1, Input", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Input", rightMotor2.get());
    }

    public void displayDriverControllerAxes() {
        SmartDashboard.putNumber("Left Trigger", OI.driverController.getLeftTriggerAxis());
        SmartDashboard.putNumber("Right Trigger", OI.driverController.getRightTriggerAxis());
        SmartDashboard.putNumber("Right - Left", OI.driverController.getRightTriggerAxis() - OI.driverController.getLeftTriggerAxis());
    }
}