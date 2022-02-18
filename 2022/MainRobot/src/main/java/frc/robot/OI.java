package frc.robot;


import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static XboxController driverController = new XboxController(0);
    public static XboxController operatController = new XboxController(1);

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
    public static boolean manualForwardsIntake() {
        return operatController.getRawButton(B_BUTTON);
    }

    public static boolean manualReverseIntake() {
        return operatController.getRawButton(X_BUTTON);
    }

    public static boolean shootLow() { 
        return (operatController.getRightTriggerAxis() > .5 || operatController.getLeftTriggerAxis() > .5);
    }

    public static boolean shootHigh() { 
        return (operatController.getRightTriggerAxis() > .5 && operatController.getLeftTriggerAxis() > .5);
     }

    public static boolean hang() {
        return false;
    }

    public static boolean pixyAutopilot() {
        return (driverController.getLeftTriggerAxis() >= 0.5);
    }

    public static boolean shooterManualOverride() {
        return (operatController.getRawButton(START_BUTTON));
    }

    /** What are you doing? it's time to stop. */
    public static String ramp() {
        return "what the heck is wrong with you. delete this code NOW";
    }

    public static boolean moveIndexDown() {
        return operatController.getRawButton(A_BUTTON);
    }

    public static boolean moveIndexUp() {
        return operatController.getRawButton(Y_BUTTON);
    }

    public static boolean collectionAssist() {
        return driverController.getRawButton(SELECT_BUTTON) == true;
    }

    public static double driveThrottle() {
        return driverController.getRawAxis(RIGHT_THUMB_VERTICAL);
    }

    public static double driveRotation() {
        return driverController.getRawAxis(LEFT_THUMB_HORIZONTAL);
    }

    public static double shooterThrottle() {
        return operatController.getRawAxis(LEFT_THUMB_VERTICAL);
    }
}