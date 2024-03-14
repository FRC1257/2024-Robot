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
        if(logP.get() != io.getP()) {
            io.setP(logP.get());
        }
        if(logI.get() != io.getI()) {
            io.setI(logI.get());
        }
        if(logD.get() != io.getD()) {
            io.setD(logD.get());
        }
        Logger.processInputs("Intake", inputs);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    public boolean isIntaked(){
        return io.isIntaked();
    }

    //replace with whatever you want
    // Dependence on pivot angle and shooter break beam
    public Command IntakeLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(voltage),
            (stop) -> setVoltage(0.0),
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
            this::isIntaked,
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
