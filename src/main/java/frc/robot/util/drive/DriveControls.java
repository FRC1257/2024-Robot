package frc.robot.util.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.pivotArm.PivotArmConstants;

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
    public static Trigger DRIVE_TOGGLE_ROBOT_RELATIVE;
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

    // Intake Controls
    public static Trigger INTAKE_IN;
    public static Trigger INTAKE_OUT;
    public static DoubleSupplier INTAKE_ROTATE;

    // Ground Intake
    public static Trigger GROUND_INTAKE_IN;
    public static Trigger GROUND_INTAKE_OUT;
    public static DoubleSupplier GROUND_INTAKE_ROTATE;

    // Shooter Controls
    public static DoubleSupplier 
    SHOOTER_SPEED;
    public static Trigger SHOOTER_PREP;
    public static Trigger SHOOTER_FIRE_AMP;
    public static Trigger SHOOTER_FIRE_SPEAKER;
    public static Trigger SHOOTER_SHOOT;
    public static Trigger SHOOTER_FULL_SEND;

    // Rumble Controls
    public static Trigger TIMED_RUMBLE;
    public static Trigger INTAKE_RUMBLE;

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
                DRIVE_TOGGLE_ROBOT_RELATIVE = EMPTY_TRIGGER;

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
                DRIVE_TOGGLE_ROBOT_RELATIVE = EMPTY_TRIGGER;
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
                PIVOT_ROTATE = operator::getRightY;
                PIVOT_AMP = operator.rightBumper();
                PIVOT_ZERO = operator.leftBumper();
                PIVOT_TO_SPEAKER = operator.a();
                
                INTAKE_IN = operator.rightTrigger();
                INTAKE_OUT = EMPTY_TRIGGER;
                INTAKE_ROTATE = operator::getLeftY;

                GROUND_INTAKE_IN = operator.y();
                GROUND_INTAKE_OUT = operator.b();
                GROUND_INTAKE_ROTATE = operator::getRightY;

                SHOOTER_SPEED = operator::getRightY;
                SHOOTER_PREP = operator.rightBumper();
                SHOOTER_FIRE_AMP = operator.leftBumper();
                SHOOTER_FIRE_SPEAKER = operator.x();
                TIMED_RUMBLE = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND = EMPTY_TRIGGER;
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
                SHOOTER_FIRE_AMP = EMPTY_TRIGGER;
                SHOOTER_FIRE_SPEAKER = operator.getY();// commented out for testing
                SHOOTER_SHOOT = EMPTY_TRIGGER;
                SHOOTER_FULL_SEND = EMPTY_TRIGGER;
                break;

                //bottom right Left joystick to intake 
        }
    }
}
