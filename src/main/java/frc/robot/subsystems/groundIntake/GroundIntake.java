package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GroundIntake.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();
    
    private LoggedDashboardNumber logP;
    private LoggedDashboardNumber logI;
    private LoggedDashboardNumber logD;
    
    public GroundIntake (GroundIntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);

        logP = new LoggedDashboardNumber("GroundIntake/P", io.getP());
        logI = new LoggedDashboardNumber("GroundIntake/I", io.getI());
        logD = new LoggedDashboardNumber("GroundIntake/D", io.getD());
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
        Logger.processInputs("GroundIntake", inputs);
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
    public Command GroundIntakeLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(voltage),
            (stop) -> setVoltage(0.0),
            this::isIntaked,
            this
        );
    }

    /**
     * Same dependence as GroundIntakeLoopCommand
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

    public Command GroundIntakeSpeedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setSpeed(speed.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command GroundIntakeManualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(voltage.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }



}
