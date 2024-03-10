package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSnailController extends CommandXboxController {
    public CommandSnailController(int port) {
        super(port);
    }

    // Configure your own special inputs to modify controls
    public double getDriveForward() {
        if (a().getAsBoolean()) {
            return -applyDeadband(getLeftY());
        } else if (rightBumper().getAsBoolean()) {
            return -applyDeadband(getRightY());
        } else if (leftTrigger().getAsBoolean()) {
            return -applyDeadband(getLeftY());
        }
        return 0;
    }

    public double getDriveTurn() {
        if (a().getAsBoolean()) {
            return -applyDeadband(getLeftX());
        } else if (rightBumper().getAsBoolean()) {
            return -applyDeadband(getLeftX());
        } else if (leftTrigger().getAsBoolean()) {
            return -applyDeadband(getRightX());
        }
        return 0;
    }

    public double getElevatorSpeed() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }

    // add extra getters
    public Trigger getX() {
        return x();
    }

    public Trigger getY() {
        return y();
    }

    public Trigger getA() {
        return a();
    }

    public Trigger getB() {
        return b();
    }

    public enum DPad {
        UP,
        RIGHT,
        DOWN,
        LEFT
    }

    public void rumble(int value) {
        System.out.println("RUMBLE");
        getHID().setRumble(RumbleType.kLeftRumble, value);
        getHID().setRumble(RumbleType.kRightRumble, value);
    }


    public Trigger getDPad(DPad dpad) {
        int angle;
        switch(dpad) {
            case UP:
                angle = 0;
                break;
            case RIGHT:
                angle = 90;
                break;
            case DOWN:
                angle = 180;
                break;
            case LEFT:
                angle = 270;
                break;
            default:
                angle = 0;
        }

        return new Trigger(() -> (this.getHID().getPOV() == angle));
    }

    //these two commands work together to get input from the joystick to control the robot
    //since getLeftBumper is true for driveforward and driveturn, it'll make it so tha tthe
    //left y joystick controls the full movement

    public static double applyDeadband(double value) {
        if (Math.abs(value) < 0.08) return 0;
        else return value;
    }

    public double getRightYD() {
        return applyDeadband(getRightY());
    }

    public double getLeftYD() {
        return applyDeadband(getLeftY());
    }

    public double getRightXD() {
        return applyDeadband(getRightX());
    }

    public double getLeftXD() {
        return applyDeadband(getLeftX());
    }
}