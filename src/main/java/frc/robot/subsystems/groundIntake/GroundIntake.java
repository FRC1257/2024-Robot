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
    
    public GroundIntake (GroundIntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Ground Intake", inputs);
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
    public Command IntakeLoopCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(voltage),
            (stop) -> setVoltage(0.0),
            this::isIntaked,
            this
        );
    }
    

}
