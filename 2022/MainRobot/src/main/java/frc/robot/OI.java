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
    public static final int D_PAD_UP = operatController.getPOV(0);
    public static final int D_PAD_RIGHT = operatController.getPOV(90);
    public static final int D_PAD_DOWN = operatController.getPOV(180);
    public static final int D_PAD_LEFT = operatController.getPOV(270);
    public static final int LEFT_THUMB_HORIZONTAL = 0;
    public static final int LEFT_THUMB_VERTICAL = 1;
    public static final int RIGHT_THUMB_HORIZONTAL = 4;
    public static final int RIGHT_THUMB_VERTICAL = 5;
    


    // Get button functions
    public static boolean manualIntake() {
        return operatController.getRawButton(B_BUTTON);
    }

    public static boolean shoot() {
        return operatController.getRawButton(A_BUTTON);
    }

    public static boolean hang() {
        return false;
    }

    public static boolean moveIndexDown() {
        return operatController.getRawButton(Y_BUTTON) == true && operatController.getPOV() == D_PAD_DOWN;
    }

    public static boolean moveIndexUp() {
        return operatController.getRawButton(Y_BUTTON) == true && operatController.getPOV() == D_PAD_UP;
    }

    public static boolean collectionAssist() {
        return driverController.getRawButton(SELECT_BUTTON) == true;
    }

    public static double driveThrottle() {
        return driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis();
    }

    public static double driveRotation() {
        return driverController.getRawAxis(RIGHT_THUMB_HORIZONTAL);
    }


}