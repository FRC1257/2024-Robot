package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase; 
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Need to import Constants files/classes */

public class GroundIntakeIOSparkMax implements GroundIntakeIO {
    
    private CANSparkMax groundIntakeMotor;

    private DigitalInput breakBeamSensor;

    public GroundIntakeIOSparkMax() {
        /** ID needs to be assigned from constants */
        groundIntakeMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        groundIntakeMotor.restoreFactoryDefaults();
        groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        groundIntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        /** ID needs to be assigned from constants */
        breakBeamSensor = new DigitalInput(0);
    }
    
    /** updates inputs from robot */
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.appliedVoltage = groundIntakeMotor.getAppliedOutput() * groundIntakeMotor.getBusVoltage();
        inputs.currentAmps = new double[] {groundIntakeMotor.getOutputCurrent()};
        inputs.tempCelcius = new double[] {groundIntakeMotor.getMotorTemperature()};
    }
    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        groundIntakeMotor.setVoltage(voltage);
    }
    /** Gets break beam state */
    @Override
    public void breakBeamSensor() {
        SmartDashboard.putBoolean("Ground Intake Break Beam", breakBeamSensor.get());
    }
    /** sets brake mode to stop */
    @Override
    public void setBrake(boolean brake) {
        groundIntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
