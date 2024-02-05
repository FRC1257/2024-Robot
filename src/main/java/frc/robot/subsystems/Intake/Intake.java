package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    public Intake (IntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    public void periodic() {
        io.updateInputs(inputs);
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
        );
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
        );
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

}
