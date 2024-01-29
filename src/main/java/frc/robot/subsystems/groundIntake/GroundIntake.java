package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GroundIntake.*;
import static frc.robot.Constants.GroundIntake.GROUND_INTAKE_PID;


public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();
    // These logged dash may need to be changed; PID part should be defined in Constants.java
    private LoggedDashboardNumber p = new LoggedDashboardNumber("GroundIntake/P", GROUND_INTAKE_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("GroundIntake/I", GROUND_INTAKE_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("GroundIntake/D", GROUND_INTAKE_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("GroundIntake/FF", GROUND_INTAKE_PID[3]);
    
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
    public void breakBeamSensor() {
        // io.breakBeamSensor(); Not sure how to implement this method
        SmartDashboard.putBoolean("Ground Intake Break Beam", breakBeamSensor.get());
    }
    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }
}
