package frc.robot.util.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
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
    public static Trigger DRIVE_ROBOT_RELATIVE;
    public static Trigger LOCK_ON_SPEAKER_FULL;


    public static Trigger DRIVE_SPEAKER_AIM;
    public static Trigger DRIVE_NOTE_GOTO;

    // Drive Trajectories
    public static Trigger DRIVE_AMP;
    public static Trigger DRIVE_SOURCE;

    // Drive Turns
    public static Trigger TURN_90;
    public static Trigger TURN_180;

    // Pivot Controls
    public static DoubleSupplier PIVOT_ROTATE;
    public static DoubleSupplier PIVOT_PID_ROTATE;
    public static Trigger PIVOT_AMP;
    public static Trigger PIVOT_ZERO;
    public static Trigger PIVOT_TO_SPEAKER;
    public static Trigger PIVOT_HOLD;

    // Intake Controls
    public static Trigger INTAKE_IN;
    public static Trigger INTAKE_OUT;
    public static DoubleSupplier INTAKE_ROTATE;

    // Ground Intake
    public static Trigger GROUND_INTAKE_IN;
    public static Trigger GROUND_INTAKE_OUT;
    public static DoubleSupplier GROUND_INTAKE_ROTATE;

    // Shooter Controls
    public static DoubleSupplier SHOOTER_SPEED;
    public static Trigger SHOOTER_PREP;
    public static Trigger SHOOTER_FULL_SEND_INTAKE;
    public static Trigger SHOOTER_FIRE_SPEAKER;
    public static Trigger SHOOTER_SHOOT;
    public static Trigger SHOOTER_FULL_SEND;
    public static Trigger SHOOTER_UNJAM;

    // Rumble Controls
    public static Trigger TIMED_RUMBLE;
    public static Trigger INTAKE_RUMBLE;

    // Potential Hail Marry Program [Suggested by Owen]
    public static Trigger SHOOT_FROM_SOURCE; 

    // Setup the controls
    public static void configureControls() {
        switch (Constants.driver) {
            case MAUI:
                DRIVE_FORWARD = () -> -(driver.getLeftY());
                DRIVE_STRAFE = ()->-(driver.getLeftX());
                DRIVE_ROTATE = () -> -(driver.getRightX());
                DRIVE_SLOW = driver.start();
                DRIVE_SPEAKER_AIM = driver.leftBumper();
                DRIVE_STOP = driver.x();
                DRIVE_ROBOT_RELATIVE = driver.rightBumper();
                LOCK_ON_SPEAKER_FULL = driver.y();

                DRIVE_AMP = EMPTY_TRIGGER;
                DRIVE_SOURCE = EMPTY_TRIGGER;
                TURN_90 = EMPTY_TRIGGER;
                TURN_180 = EMPTY_TRIGGER;
                LOCK_ON_SPEAKER_FULL = EMPTY_TRIGGER;
                break;
            case PROGRAMMERS:
            default:
                DRIVE_FORWARD = () -> -driver.getLeftY();
                DRIVE_STRAFE = () -> -driver.getLeftX();
                DRIVE_ROTATE = driver::getRightX;
                DRIVE_SLOW = driver.x();
                DRIVE_SPEAKER_AIM = driver.b();
                DRIVE_NOTE_GOTO = EMPTY_TRIGGER;
                DRIVE_STOP = driver.rightBumper();
                DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;
                LOCK_ON_SPEAKER_FULL = EMPTY_TRIGGER;

                DRIVE_AMP = driver.leftBumper();
                DRIVE_SOURCE = driver.a();
                TURN_90 = driver.y();
                TURN_180 = driver.start();
                break;
        }

        switch (Constants.operator) {
            case ERICK:
                // Operator controls
                PIVOT_ROTATE = () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                PIVOT_PID_ROTATE = EMPTY_DOUBLE_SUPPLIER; //() -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                //isn't reading operator.getLeftTriggerAxis, must be an issue with the encoder
                
                // Pivot things
                PIVOT_AMP = operator.getDPad(DPad.RIGHT);
                PIVOT_ZERO = operator.getDPad(DPad.DOWN);
                PIVOT_TO_SPEAKER = operator.getDPad(DPad.LEFT);
                PIVOT_HOLD = operator.getDPad(DPad.UP);
                
                // intaking things
                INTAKE_IN = operator.rightBumper();
                INTAKE_OUT = operator.leftBumper();
                INTAKE_ROTATE = () -> operator.getLeftYD();

                GROUND_INTAKE_IN = operator.rightBumper();
                GROUND_INTAKE_OUT = operator.leftBumper();
                GROUND_INTAKE_ROTATE = () -> -2*operator.getLeftXD();

                // SHOOTER DEFAULT SPEED
                SHOOTER_SPEED = () -> operator.getRightXD();
                //SHOOTER_SPEED = operator::getRightTriggerAxis;\
                SHOOTER_PREP = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND_INTAKE = operator.getX();
                SHOOTER_FIRE_SPEAKER = EMPTY_TRIGGER;// commented out for testing
                SHOOTER_SHOOT = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND = operator.getA();
                SHOOTER_UNJAM = operator.getB();
                SHOOT_FROM_SOURCE = EMPTY_TRIGGER;
                break;
            case PROGRAMMERS:
            default:
                // Operator controls
                PIVOT_ROTATE = () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                PIVOT_PID_ROTATE = () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
                //isn't reading operator.getLeftTriggerAxis, must be an issue with the encoder
                PIVOT_AMP = operator.getB();
                PIVOT_ZERO = operator.getA();
                PIVOT_TO_SPEAKER = EMPTY_TRIGGER;
                
                INTAKE_IN = operator.rightBumper();
                INTAKE_OUT = operator.leftBumper();
                INTAKE_ROTATE = () -> operator.getLeftYD();

                GROUND_INTAKE_IN = operator.rightBumper();
                GROUND_INTAKE_OUT = operator.leftBumper();
                GROUND_INTAKE_ROTATE = () -> -2*operator.getLeftXD();

                SHOOTER_SPEED = operator::getRightXD;
                //SHOOTER_SPEED = operator::getRightTriggerAxis;\
                SHOOTER_PREP = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND_INTAKE = EMPTY_TRIGGER;
                SHOOTER_FIRE_SPEAKER = operator.getY();// commented out for testing
                SHOOTER_SHOOT = EMPTY_TRIGGER;
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
}
