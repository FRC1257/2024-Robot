package frc.robot.util.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.drive.CommandSnailController.DPad;

public class DriveControls {
    // Controllers
    public static final CommandSnailController driver = new CommandSnailController(0);
    public static final CommandSnailController operator = new CommandSnailController(1);

    // Useful for things that don't need to be triggered
    private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
    private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

    // Drive controls
    public static DoubleSupplier DRIVE_FORWARD;
    public static DoubleSupplier DRIVE_STRAFE;
    public static DoubleSupplier DRIVE_ROTATE;
    public static Trigger DRIVE_SLOW;
    public static Trigger DRIVE_STOP;

    // drive modes
    public static Trigger DRIVE_ROBOT_RELATIVE;
    public static Trigger DRIVE_SPEAKER_AIM;

    // Drive Angle Locks
    public static Trigger LOCK_BACK;
    public static Trigger LOCK_PICKUP;
    public static Trigger LOCK_PASS;
    public static Trigger LOCK_ON_AMP;

    // Drive Trajectories
    public static Trigger DRIVE_AMP;

    // Pivot Controls
    public static DoubleSupplier PIVOT_ROTATE;
    public static Trigger PIVOT_AMP;
    public static Trigger PIVOT_ZERO;
    public static Trigger PIVOT_TO_SPEAKER;
    public static Trigger PIVOT_PODIUM;
    public static Trigger PIVOT_ANYWHERE;

    // Intake Controls
    public static Trigger INTAKE_IN;
    public static Trigger INTAKE_OUT;
    public static DoubleSupplier INTAKE_ROTATE;

    // Ground Intake
    public static Trigger GROUND_INTAKE_IN;
    public static Trigger GROUND_INTAKE_OUT;
    public static DoubleSupplier GROUND_INTAKE_ROTATE;

    public static Trigger INTAKE_UNTIL_INTAKED;

    // Shooter Controls
    public static DoubleSupplier SHOOTER_SPEED;
    public static Trigger SHOOTER_FULL_SEND_INTAKE;
    public static Trigger SHOOTER_FULL_SEND;
    public static Trigger SHOOTER_UNJAM;

    // Setup the controls
    public static void configureControls() {
        switch (Constants.driver) {
            case MAUI:
                // Driver controls
                DRIVE_FORWARD = () -> (-driver.getLeftY());
                DRIVE_STRAFE = ()->(-driver.getLeftX());
                DRIVE_ROTATE = () -> (-driver.getRightX());
                
                // Driver Settings
                DRIVE_SLOW = driver.start();
                DRIVE_STOP = driver.x();

                // Driver Modes
                DRIVE_ROBOT_RELATIVE = driver.y();
                DRIVE_SPEAKER_AIM = driver.leftBumper(); // uses vision

                // Driver Angle Locks
                LOCK_BACK = driver.getDPad(DPad.DOWN);
                LOCK_PICKUP = driver.getDPad(DPad.RIGHT);
                LOCK_ON_AMP = driver.rightBumper();
                LOCK_PASS = driver.getDPad(DPad.LEFT); // uses vision

                DRIVE_AMP = driver.b(); // uses vision
                break;
            case PROGRAMMERS:
            default:
                // Driver controls
                DRIVE_FORWARD = () -> (-driver.getLeftY());
                DRIVE_STRAFE = ()->(-driver.getLeftX());
                DRIVE_ROTATE = () -> (-driver.getRightX());
                
                // Driver Settings
                DRIVE_SLOW = driver.start();
                DRIVE_STOP = driver.x();

                // Driver Modes
                DRIVE_ROBOT_RELATIVE = driver.y();
                DRIVE_SPEAKER_AIM = driver.b(); // uses vision

                // Driver Angle Locks
                LOCK_BACK = driver.getDPad(DPad.DOWN);
                LOCK_PICKUP = driver.getDPad(DPad.RIGHT);
                LOCK_ON_AMP = driver.rightBumper();
                LOCK_PASS = driver.getDPad(DPad.LEFT); // uses vision

                DRIVE_AMP = EMPTY_TRIGGER; // uses vision
        }

        switch (Constants.operator) {
            case ERICK:
                PIVOT_ROTATE = () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                
                // Pivot things
                PIVOT_AMP = operator.getDPad(DPad.RIGHT);
                PIVOT_ZERO = operator.getDPad(DPad.DOWN);
                PIVOT_TO_SPEAKER = operator.getDPad(DPad.LEFT);
                PIVOT_PODIUM = EMPTY_TRIGGER;
                PIVOT_ANYWHERE = operator.getDPad(DPad.UP); // uses vision
                
                // intaking things
                INTAKE_ROTATE = () -> operator.getLeftYD();
                INTAKE_IN = operator.rightBumper();
                INTAKE_OUT = operator.leftBumper();
                INTAKE_UNTIL_INTAKED = operator.getY();

                // ground intake things
                GROUND_INTAKE_ROTATE = () -> -2*operator.getLeftXD();
                GROUND_INTAKE_IN = operator.rightBumper();
                GROUND_INTAKE_OUT = operator.leftBumper();

                // Shooter things
                SHOOTER_SPEED = () -> operator.getRightXD();
                SHOOTER_FULL_SEND_INTAKE = operator.getX();
                SHOOTER_FULL_SEND = operator.getA();
                SHOOTER_UNJAM = operator.getB();
                break;
            case PROGRAMMERS:
            default:
                // Operator controls
                PIVOT_ROTATE = () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                
                //isn't reading operator.getLeftTriggerAxis, must be an issue with the encoder
                PIVOT_AMP = operator.getB();
                PIVOT_ZERO = operator.getA();
                PIVOT_TO_SPEAKER = EMPTY_TRIGGER;
                PIVOT_PODIUM = operator.getDPad(DPad.UP);
                
                INTAKE_IN = operator.rightBumper();
                INTAKE_OUT = operator.leftBumper();
                INTAKE_ROTATE = () -> operator.getLeftYD();

                GROUND_INTAKE_IN = operator.rightBumper();
                GROUND_INTAKE_OUT = operator.leftBumper();
                GROUND_INTAKE_ROTATE = () -> -2*operator.getLeftXD();

                SHOOTER_SPEED = operator::getRightXD;

                SHOOTER_FULL_SEND_INTAKE = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND = EMPTY_TRIGGER;
                SHOOTER_UNJAM = EMPTY_TRIGGER;
                break;

                //bottom right Left joystick to intake 
        }
    }

    private static Command getRumbleCommand(CommandSnailController driver) {
        return new InstantCommand(() -> driver.rumble(1)).andThen(new WaitCommand(1)).andThen(() -> driver.rumble(0));
    }

    public static Command getRumbleBoth() {
        return getRumbleCommand(driver).alongWith(getRumbleCommand(operator));
    }

    public static Command getRumbleOperator() {
        return getRumbleCommand(operator);
    }

    public static Command getRumbleDriver() {
        return getRumbleCommand(driver);
    }
}
