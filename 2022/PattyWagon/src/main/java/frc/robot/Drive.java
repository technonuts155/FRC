package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive {
    // Motor time
    private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.leftMotor1, MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.leftMotor2, MotorType.kBrushless);
    private CANSparkMax leftMotor3 = new CANSparkMax(RobotMap.leftMotor3, MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.rightMotor1, MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.rightMotor2, MotorType.kBrushless);
    private CANSparkMax rightMotor3 = new CANSparkMax(RobotMap.rightMotor3, MotorType.kBrushless);

    // Group left and right motors
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

    // Create drivetrain object
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pixycam
    private Pixy2 pixy;
    private final double HORIZONTAL_CENTER = 157.5;

    // PIDController for centering on target found by pixycam
    private PIDController drivePID = new PIDController(0.005, 0, 0);

    public void resetPID() {
        drivePID.reset();
    }

    // Used for tuning the PID
    // Pulls numbers from the Preferences box of the Smartdashboard
    public void updatePIDValues() {
        drivePID.setP(Preferences.getDouble("drivePID kP", 0));
        drivePID.setI(Preferences.getDouble("drivePID kI", 0));
        drivePID.setD(Preferences.getDouble("drivePID kD", 0));
        drivePID.setTolerance(Preferences.getDouble("drivePID Tolerance", 10));
        drivePID.setSetpoint(Preferences.getDouble("drivePID Setpoint", 157.5));
    }

    public void setPIDSetpoint(double setpoint) {
        drivePID.setSetpoint(setpoint);
    }

    public void setPIDTolerance(double tolerance) {
        drivePID.setTolerance(tolerance);
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

    public void pixyAutopilot() {
        Block target = getTargetBlock();

        if (target != null) {
            double turnRate = drivePID.calculate(getBlockCenterX(target), HORIZONTAL_CENTER);
            SmartDashboard.putNumber("Error", getBlockCenterX(target) - HORIZONTAL_CENTER);
            SmartDashboard.putNumber("Turn rate", turnRate);
            drivetrain.arcadeDrive(0, turnRate);
        } else {
            drivetrain.arcadeDrive(0, 0);
        }

    }

    public void PIDAtSetpoint() {
        SmartDashboard.putBoolean("At setpoint", drivePID.atSetpoint());
    }

    public void displayPIDValues() {
        SmartDashboard.putNumber("P", drivePID.getP());
        SmartDashboard.putNumber("I", drivePID.getI());
        SmartDashboard.putNumber("D", drivePID.getD());
    }

    public void XboxDrive() {

        // Read controller values
        double rightTriggerAxis = OI.driveController.getRightTriggerAxis();
        double leftTriggerAxis = OI.driveController.getLeftTriggerAxis();
        double speed = rightTriggerAxis - leftTriggerAxis;
        double rotation = OI.driveController.getRawAxis(4);

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
        drivetrain.arcadeDrive(speed, rotation * .75);
    }

    public void displayMotorControllerOutputCurrents() {
        // Displays output currents for each speed controller (in amps)
        SmartDashboard.putNumber("Left Drive 1, Current", leftMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Left Drive 2, Current", leftMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Left Drive 3, Current", leftMotor3.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 1, Current", rightMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 2, Current", rightMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive 3, Current", rightMotor3.getOutputCurrent());
    }

    public void displayMotorControllerInputs() {
        // Displays the input given to each speed controller (range of -1 -> 1)
        SmartDashboard.putNumber("Left Drive 1, Input", leftMotor1.get());
        SmartDashboard.putNumber("Left Drive 2, Input", leftMotor2.get());
        SmartDashboard.putNumber("Left Drive 3, Input", leftMotor3.get());
        SmartDashboard.putNumber("Right Drive 1, Input", rightMotor1.get());
        SmartDashboard.putNumber("Right Drive 2, Input", rightMotor2.get());
        SmartDashboard.putNumber("Right Drive 3, Input", rightMotor3.get());
    }

    public void displayDriveControllerAxes() {
        SmartDashboard.putNumber("Left Trigger", OI.driveController.getLeftTriggerAxis());
        SmartDashboard.putNumber("Right Trigger", OI.driveController.getRightTriggerAxis());
        SmartDashboard.putNumber("Right - Left", OI.driveController.getRightTriggerAxis() - OI.driveController.getLeftTriggerAxis());
    }
}
