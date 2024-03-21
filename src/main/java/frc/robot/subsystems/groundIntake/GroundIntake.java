package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants.*;
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

        // Allows manual command of the flywheel for testing
    public Command manualCommand(double voltage) {
        return manualCommand(() -> voltage);
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
