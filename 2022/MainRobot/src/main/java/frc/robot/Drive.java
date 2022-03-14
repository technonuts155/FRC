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

    // Motor Controllers
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
    // private final double HORIZONTAL_CENTER = 157.5;

    // PIDControllers
    private PIDController pixyPID = new PIDController(0.015, 0.0, 0.001);
    private PIDController encoderPIDLeft = new PIDController(0.01, 0.0005, 0);
    private PIDController encoderPIDRight = new PIDController(0.01, 0.0005, 0);

    // Encoders
    private Encoder leftDriveEncoder = new Encoder(RobotMap.LEFT_DRIVE_ENCODER_A, RobotMap.LEFT_DRIVE_ENCODER_B);
    private Encoder rightDriveEncoder = new Encoder(RobotMap.RIGHT_DRIVE_ENCODER_A, RobotMap.RIGHT_DRIVE_ENCODER_B);
    private final double PULSES_TO_INCHES = 1 / 18.9231;

    // Add more zeros to this to decrease throttle ramp rate
    private double[] inputHistory = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int inputIndex = 0;

    public Drive() {
        leftDriveEncoder.setDistancePerPulse(PULSES_TO_INCHES);
        rightDriveEncoder.setDistancePerPulse(PULSES_TO_INCHES);

        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();

        leftMotors.setInverted(true);
    }

    /** ---------- Encoders & Encoder Drive Methods ---------- */

    public void encoderPIDDrive() {
            rightMotors.set(encoderPIDRight.calculate(getRightEncoderDistance(), 0));
            leftMotors.set(encoderPIDLeft.calculate(getLeftEncoderDistance(), 0));
    }

    public void resetEncoders() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
    }

    public double getLeftEncoderDistance(){
        return leftDriveEncoder.getDistance();
    }

    public double getRightEncoderDistance(){
        return rightDriveEncoder.getDistance();
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
        speed = linearRamp(speed);

        if (target != null) {
            double turnRate = pixyPID.calculate(getBlockCenterX(target), 190);
            drivetrain.arcadeDrive(speed, turnRate);
        } else {
            drivetrain.arcadeDrive(speed, OI.driveRotation());
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
        pixyPID.setP(Preferences.getDouble("pixyPID kP", 0.0));
        pixyPID.setI(Preferences.getDouble("pixyPID kI", 0.0));
        pixyPID.setD(Preferences.getDouble("pixyPID kD", 0.0));
        pixyPID.setTolerance(Preferences.getDouble("pixyPID Tolerance", 10));
        pixyPID.setSetpoint(Preferences.getDouble("pixyPID Setpoint", 157.5));
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

        // Getting block cache doesn't work if you don't get block count first
        int blocksFound;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            blocksFound = pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG1, 10);
        } else {
            blocksFound = pixy.getCCC().getBlocks(false, pixy.getCCC().CCC_SIG2, 10);
        }

        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        ArrayList<Block> ratioedBlocks = new ArrayList<Block>();

        // Create new list of only blocks that are square enough
        // Perfectly identified cargo should be a square because
        // a bounding box drawn around a circle is a square.
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
        drivetrain.arcadeDrive(linearRamp(OI.driveThrottle()), OI.driveRotation());
    }

    public void stop() {
        drivetrain.arcadeDrive(0, 0);
    }

    /** 
     * Averages the input values for the last ~200ms.
     * Results in a linear acceleration
     */
    private double linearRamp(double input) {
        inputHistory[inputIndex % inputHistory.length] = input;
        double total = 0;
        for (int i = 0; i < inputHistory.length; i++) {
            total = total + inputHistory[i];
        }
        inputIndex++;
        return total / inputHistory.length;
    }

    public void displayMotorControllerInputs() {
        // Displays the input given to each speed controller (range of -1 -> 1)
        SmartDashboard.putNumber("Left Drive 1, Input", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Input", leftMotor2.get());
        SmartDashboard.putNumber("Right Drive 1, Input", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Input", rightMotor2.get());
    }
}