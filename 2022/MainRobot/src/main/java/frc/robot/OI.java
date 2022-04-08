package frc.robot;


import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static XboxController driverController = new XboxController(0);
    public static XboxController operatorController = new XboxController(1);

    // Defining button IDs
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int RIGHT_BUTTON = 5;
    public static final int LEFT_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_STICK_PRESS_BUTTON = 9;
    public static final int RIGHT_STICK_PRESS_BUTTON = 10;
    public static final int D_PAD_UP = 0;
    public static final int D_PAD_RIGHT = 90;
    public static final int D_PAD_DOWN = 180;
    public static final int D_PAD_LEFT = 270;
    public static final int LEFT_THUMB_HORIZONTAL = 0;
    public static final int LEFT_THUMB_VERTICAL = 1;
    public static final int RIGHT_THUMB_HORIZONTAL = 4;
    public static final int RIGHT_THUMB_VERTICAL = 5;
    public static final int RIGHT_ANALOG_TRIGGER = 3;
    public static final int LEFT_ANALOG_TRIGGER = 2;

    


    // Get button functions
    public static boolean intakeOut() {
        return operatorController.getRawButton(B_BUTTON);
    }

    public static boolean intakeIn() {
        return operatorController.getRawButton(X_BUTTON);
    }

    // ^ operator is a 'XOR' operator, exclusive-or. Meaning it will return true if ONLY one condition is met, not both.
    // In this case, shootLow will return true if either trigger is held, but not when both are held.
    public static boolean shootStaticRPM() { 
        return (operatorController.getRightTriggerAxis() > .5 ^ operatorController.getLeftTriggerAxis() > .5);
    }

    public static boolean shootDynamicRPM() { 
        return (operatorController.getRightTriggerAxis() > .5 && operatorController.getLeftTriggerAxis() > .5);
     }

    public static boolean pixyAssistedDrive() {
        return driverController.getLeftBumper();
    }

    public static boolean shooterManualOverride() {
        return (operatorController.getRawButton(START_BUTTON));
    }

    /** What are you doing? it's time to stop. */
    public static String ramp() {
        return "what the heck is wrong with you. delete this code NOW";
    }

    public static boolean moveIndexDown() {
        return operatorController.getRawButton(A_BUTTON);
    }

    public static boolean moveIndexUp() {
        return operatorController.getRawButton(Y_BUTTON);
    }

    public static double driveThrottle() {
        double speed = driverController.getRawAxis(LEFT_THUMB_VERTICAL);

        if (speed < 0) {
            speed = speed * speed * -1;
        } else {
            speed = speed * speed;
        }

        return speed;
    }

    // Make sure this is inverted here
    public static double driveRotation() {
        double rotation = driverController.getRawAxis(RIGHT_THUMB_HORIZONTAL);

        if (rotation < 0) {
            rotation = rotation * rotation * -1;
        } else {
            rotation = rotation * rotation; 
        }

        return -rotation * .7;
    }

    public static double shooterThrottle() {
        return operatorController.getRawAxis(LEFT_THUMB_VERTICAL);
    }
    
    public static boolean climbExtend() {
        return operatorController.getPOV() == D_PAD_UP;
    }

    public static boolean climbRetract() {
        return operatorController.getPOV() == D_PAD_DOWN;
    }

    public static boolean climbLock() {
        return operatorController.getLeftBumper();
    }

    public static boolean climbUnlock() {
        return operatorController.getRightBumper();
    }

    public static boolean hubAssistedDrive() {
        return driverController.getRightBumper();
    }
}