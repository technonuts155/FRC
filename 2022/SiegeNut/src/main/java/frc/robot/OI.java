package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static XboxController driveController = new XboxController(0);

    public static int A_BUTTON = 1;
    public static int B_BUTTON = 2;
    public static int X_BUTTON = 3;
    public static int Y_BUTTON = 4;
    public static int LEFT_BUMPER = 5;
    public static int RIGHT_BUMPER = 6;
    public static int START_BUTTON = 7;
    public static int SELECT_BUTTON = 8;
    public static int LEFT_JOYSTICK_DOWN = 9;
    public static int RIGHT_JOYSTICK_DOWN = 10;

    public static int LEFT_JOYSTICK_VERTICAL = 1;
    public static int RIGHT_JOYSTICK_HORIZONTAL = 4;
    public static int LEFT_TRIGGER = 2;
    public static int RIGHT_TRIGGER = 3;

    public static double getDriveThrottle() {
        return driveController.getRawAxis(LEFT_JOYSTICK_VERTICAL);
    }
    public static double getDriveRotation() {
        return driveController.getRawAxis(RIGHT_JOYSTICK_HORIZONTAL);
    }
    public static boolean intakeIn() {
        return driveController.getRawButton(X_BUTTON);
    }
    public static boolean intakeOut() {
        return driveController.getRawButton(B_BUTTON);
    }
    public static double shooterOutput() {
        return driveController.getRightTriggerAxis();
    }
    public static boolean timeToShoot() {
        return driveController.getLeftTriggerAxis() > .5;
    }

}