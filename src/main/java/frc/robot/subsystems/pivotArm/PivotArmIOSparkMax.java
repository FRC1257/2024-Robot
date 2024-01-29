package frc.robot.subsystems.pivotArm;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.PivotArm;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.PivotArm.PivotArmPhysicalConstants.*;

public class PivotArmIOSparkMax implements PivotArmIO {
    // Motor and Encoders
    private CANSparkMax pivotMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0;

    public PivotArmIOSparkMax() {
        pivotMotor = new CANSparkMax(PIVOT_ARM_ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pidController = pivotMotor.getPIDController();
        pidController.setOutputRange(-1, 1);

        encoder = pivotMotor.getEncoder();
        encoder.setPositionConversionFactor(PivotArm.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(PivotArm.POSITION_CONVERSION_FACTOR / 60);
        encoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

    @Override
    public void setPIDConstants(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        inputs.angle = encoder.getPosition();
        inputs.angleRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
    }

    /** Run open loop at the specified voltage. */
    @Override
    public void setVoltage(double motorVolts) {
        pivotMotor.setVoltage(motorVolts);
    }

    /** Returns the current distance measurement. */
    @Override
    public double getAngle() {
        return encoder.getPosition();
    }

    /** Go to Setpoint */
    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setBrake(boolean brake) {
        pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(encoder.getPosition() - setpoint) < PivotArm.PIVOT_ARM_PID_TOLERANCE;
    }

    @Override
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setI(i);
    }

    @Override
    public void setD(double d) {
        pidController.setD(d);
    }

    @Override
    public void setFF(double ff) {
        pidController.setFF(ff);
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
    public double getFF() {
        return pidController.getFF();
    }

}
