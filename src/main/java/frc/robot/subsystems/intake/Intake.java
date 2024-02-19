package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private LoggedDashboardNumber logP;
    private LoggedDashboardNumber logI;
    private LoggedDashboardNumber logD;
    
    public Intake (IntakeIO io) {
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
        ).withTimeout(5);
    }

    /**
     * Same dependence as IntakeLoopCommand
     */ 
    public Command EjectLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(-voltage), // Spins the other way
            (stop) -> setVoltage(0.0),
            this::isIntaked,
            this
        ).withTimeout(2);
    }

    public Command IntakeSpeedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setSpeed(speed.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command IntakeManualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(voltage.getAsDouble()),
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
