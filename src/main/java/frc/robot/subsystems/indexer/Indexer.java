package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    
    private LoggedDashboardNumber logP;
    private LoggedDashboardNumber logI;
    private LoggedDashboardNumber logD;

    // State of the note in the intake
    enum NoteState {
        NOT_ENOUGH, // Not far enough in the intake or not in there at all
        GOLDILOCKS, // just the right position in intake
        MIDDLE, // 
        OVERSHOOT
    }

    private NoteState noteState = NoteState.GOLDILOCKS;
    private double currentVoltage = 0;
    private double timeInIntake = 0;
    private final double desiredTimeInIntake = 0.5;
    
    public Indexer (IndexerIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
        logP = new LoggedDashboardNumber("Intake/P", io.getP());
        logI = new LoggedDashboardNumber("Intake/I", io.getI());
        logD = new LoggedDashboardNumber("Intake/D", io.getD());
    }

    public void periodic() {
        io.updateInputs(inputs);
        // Update PID constants to ensure they are up to date
        Logger.processInputs("Intake", inputs);

        // Updates state of where the note is in the intake
        if(noteState == NoteState.NOT_ENOUGH && inputs.breakBeam) { // Note just entered the right spot
            noteState = NoteState.GOLDILOCKS;
        }
        else if(noteState == NoteState.GOLDILOCKS && !inputs.breakBeam) { // Note just left the right spot
            if(io.getSpeed() > 0) {
                noteState = NoteState.MIDDLE;
            } else {
                noteState = NoteState.NOT_ENOUGH;
            }
            currentVoltage = Math.max(currentVoltage - 2, 0); // Intake gets progressively slower every time you overshoot
        }
        else if(noteState == NoteState.MIDDLE && inputs.breakBeam) { // Note either overshot or got to the right spot from middle
            if(io.getSpeed() > 0) {
                noteState = NoteState.OVERSHOOT;
            } else {
                noteState = NoteState.GOLDILOCKS;
            }
        }
        else if(noteState == NoteState.OVERSHOOT && !inputs.breakBeam) { // Note is either shot out or goes to the middle
            if(io.getSpeed() > 0) {
                noteState = NoteState.NOT_ENOUGH;
            } else {
                noteState = NoteState.MIDDLE;
            }
        }

        // Updates the amount of time that the note has been in the intake for
        if(noteState == NoteState.GOLDILOCKS) {
            timeInIntake += 0.02;
        }
        else {
            timeInIntake = 0;
        }

        Logger.recordOutput("Indexer/State", noteState.name());
    }

    public void setVoltage(double voltage) {
        if(SmartDashboard.getBoolean("Turbo Mode", false)){
            io.setVoltage(0);
        } else {
            io.setVoltage(voltage);
        }
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    public boolean isIntaked() {
        return io.isIntaked();
    }

    // Sets motor speed based on where the note is in the intake
    public void runIntakeLoop() {
        setVoltage(currentVoltage);
        /* switch(noteState) {
            case NOT_ENOUGH:
                setVoltage(currentVoltage);
                break;
            case GOLDILOCKS:
                setVoltage(0);
                break;
            case MIDDLE: case OVERSHOOT:
                setVoltage(-currentVoltage);
                break;
        } */
    }

    // Checks if note has been in the intake for 0.5 seconds
    public boolean isIntakedForEnoughTime() {
        return timeInIntake >= desiredTimeInIntake;
    }

    

    //replace with whatever you want
    // Dependence on pivot angle and shooter break beam
    public Command IntakeLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> { currentVoltage = voltage; },
            this::runIntakeLoop,
            (stop) -> setVoltage(0.0),
            //this::isIntakedForEnoughTime,
            this::isIntaked,
            this
        ).withTimeout(IndexerConstants.getIntakeLoopMaxTime());
    }

    // The above command in reverse
    public Command EjectLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(-voltage), // Spins the other way
            (stop) -> setVoltage(0.0),
            () -> noteState == NoteState.NOT_ENOUGH,
            this
        ).withTimeout(IndexerConstants.getIntakeLoopMaxTime());
    }
    /**
     * Uses input from controller to set speed of the flywheel
     * and is used as the default command for the ground intake
    */
    public Command speedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setSpeed(speed.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }
    // Allows manual command of the flywheel for testing
    public Command manualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(voltage.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command manualCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(voltage),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command stop() {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(0),
            (stop) -> io.stop(),
            () -> false,
            this
        );
      }

}
