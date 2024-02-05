package frc.robot.subsystems.trapPivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.TrapPivot.TrapPivotReal.*;

public class TrapPivotIOSparkMax implements TrapPivotIO {
    // Standard WPILib classes for motor stuff
    private CANSparkMax pivotMotor;
    private SparkPIDController pidController;
    private SparkAbsoluteEncoder encoder;

    private double setpoint = 0; // PID setpoint

    public TrapPivotIOSparkMax() {
        pivotMotor = new CANSparkMax(TRAP_PIVOT_ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        // Motor 
        pivotMotor.setInverted(false);
        pivotMotor.enableVoltageCompensation(12.0);
        pivotMotor.setSmartCurrentLimit(30);
        pivotMotor.burnFlash();

        configureEncoders();
        configurePID();
    }

    private void configureEncoders() {
        encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        encoder.setPositionConversionFactor(Math.PI * 2 / TRAP_PIVOT_GEARBOX_REDUCTION);
        encoder.setVelocityConversionFactor(Math.PI * 2 / TRAP_PIVOT_GEARBOX_REDUCTION / 60.0);
    }

    private void configurePID() {
        pidController = pivotMotor.getPIDController();
        pidController.setP(TRAP_PIVOT_PID_REAL[0]);
        pidController.setI(TRAP_PIVOT_PID_REAL[1]);
        pidController.setD(TRAP_PIVOT_PID_REAL[2]);
    }

    @Override
    public void updateInputs(TrapPivotIOInputs inputs) {
        inputs.angleRads = encoder.getPosition();
        inputs.angVelocityRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = new double[] { pivotMotor.getOutputCurrent() };
        inputs.tempCelsius = new double[] { pivotMotor.getMotorTemperature() };
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocity) {
        pivotMotor.set(velocity);
    }

    @Override
    public double getAngleRads() {
        return encoder.getPosition();
    }

    @Override
    public void setBrake(boolean brake) {
        pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(this.setpoint, ControlType.kPosition);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getAngleRads() - setpoint) <= PID_TOLERANCE;
    }

    @Override
    public double getP() {
        return pidController.getP();
    }

    @Override
    public double getI() {
        return pidController.getI();
    }

    @Override
    public double getD() {
        return pidController.getD();
    }

    @Override
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setP(i);
    }

    @Override
    public void setD(double d) {
        pidController.setP(d);
    }
}
