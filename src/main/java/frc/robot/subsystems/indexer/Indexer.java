package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.util.drive.DashboardValues;

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

    @AutoLogOutput(key = "Indexer/Close")
    public boolean isVoltageClose(double setVoltage) {
        double voltageDifference = Math.abs(setVoltage - inputs.appliedVoltage);
        return voltageDifference <= IndexerConstants.INDEXER_TOLERANCE;
    }

    public void periodic() {
        io.updateInputs(inputs);
        // Update PID constants to ensure they are up to date
        Logger.processInputs("Intake", inputs);

        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/State", noteState.name());
        Logger.recordOutput("Indexer/IndexerMotorConnected", inputs.velocityRadsPerSec != 0);
    }

    public void setVoltage(double voltage) {
        if (DashboardValues.turboMode.get()) {
            io.setVoltage(0);
        } else {
            io.setVoltage(voltage);
        }
        isVoltageClose(voltage);
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
