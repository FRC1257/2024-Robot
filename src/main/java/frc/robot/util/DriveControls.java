package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class DriveControls {
    // Controllers
    public static final CommandSnailController driver = new CommandSnailController(0);
    public static final CommandSnailController operator = new CommandSnailController(1);

    // Useful for things that don't need to be triggered
    private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);

    // Drive controls
    public static DoubleSupplier DRIVE_FORWARD;
    public static DoubleSupplier DRIVE_STRAFE;
    public static DoubleSupplier DRIVE_ROTATE;
    public static Trigger DRIVE_SLOW;
    public static Trigger DRIVE_STOP;

    public static Trigger DRIVE_SPEAKER_AIM;

    // Drive Trajectories
    public static Trigger DRIVE_AMP;
    public static Trigger DRIVE_SOURCE;

    // Drive Turns
    public static Trigger TURN_90;
    public static Trigger TURN_180;

    // Setup the controls
    public static void configureControls() {
        switch (Constants.driver) {
            case MAUI:
                DRIVE_FORWARD = driver::getLeftY;
                DRIVE_STRAFE = driver::getLeftX;
                DRIVE_ROTATE = driver::getRightX;
                DRIVE_SLOW = driver.x();
                DRIVE_SPEAKER_AIM = driver.leftBumper();
                DRIVE_STOP = driver.rightBumper();

                DRIVE_AMP = EMPTY_TRIGGER;
                DRIVE_SOURCE = EMPTY_TRIGGER;
                TURN_90 = EMPTY_TRIGGER;
                TURN_180 = EMPTY_TRIGGER;
                break;
            case PROGRAMMERS:
            default:
                DRIVE_FORWARD = driver::getLeftY;
                DRIVE_STRAFE = driver::getLeftX;
                DRIVE_ROTATE = driver::getRightX;
                DRIVE_SLOW = driver.x();
                DRIVE_SPEAKER_AIM = driver.leftBumper();
                DRIVE_STOP = driver.rightBumper();

                DRIVE_AMP = driver.b();
                DRIVE_SOURCE = driver.a();
                TURN_90 = driver.y();
                TURN_180 = driver.start();
                break;
        }

        switch (Constants.operator) {
            case ERICK:
                // Operator controls
                break;
            case PROGRAMMERS:
            default:
                // Operator controls
                break;
        }
    }
}
