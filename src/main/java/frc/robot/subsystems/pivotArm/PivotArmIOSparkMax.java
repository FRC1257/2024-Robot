package frc.robot.subsystems.pivotArm;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.PivotArm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.PivotArm.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

public class PivotArmIOSparkMax implements PivotArmIO {
    // Motor and Encoders
    private CANSparkMax pivotMotor, leftSlave, rightSlaveFront, rightSlaveBack;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0;

    public PivotArmIOSparkMax() {
        pivotMotor = new CANSparkMax(PIVOT_ARM_ID, MotorType.kBrushless);
        leftSlave = new CANSparkMax(LEFT_SLAVE_ID, MotorType.kBrushless);
        rightSlaveFront = new CANSparkMax(RIGHT_SLAVE_FRONT_ID, MotorType.kBrushless);
        rightSlaveBack = new CANSparkMax(RIGHT_SLAVE_BACK_ID, MotorType.kBrushless);

        pivotMotor.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightSlaveFront.restoreFactoryDefaults();
        rightSlaveBack.restoreFactoryDefaults();
        
        pivotMotor.setInverted(false);
        leftSlave.setInverted(false);
        rightSlaveFront.setInverted(true);
        rightSlaveBack.setInverted(true);

        leftSlave.follow(pivotMotor);
        rightSlaveFront.follow(pivotMotor);
        rightSlaveBack.follow(pivotMotor);

        pivotMotor.enableVoltageCompensation(12.0);
        pivotMotor.setSmartCurrentLimit(30);
        pivotMotor.burnFlash();

        configurePID();

        encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);
        encoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

    private void configureEncoders() {
        encoder = pivotMotor.getEncoder();

        encoder.setPositionConversionFactor(Math.PI * PIVOT_ARM_ROTATION_DIAM_M / PIVOT_ARM_GEARBOX_REDUCTION);
        encoder.setVelocityConversionFactor(Math.PI * PIVOT_ARM_ROTATION_DIAM_M / PIVOT_ARM_GEARBOX_REDUCTION / 60.0);

        encoder.setPosition(0);
    }

    private void configurePID() {
        pidController = pivotMotor.getPIDController();
        pidController.setOutputRange(PIVOT_ARM_MIN_ANGLE, PIVOT_ARM_MAX_ANGLE);
        pidController.setP(PIVOT_ARM_PID_REAL[0]);
        pidController.setI(PIVOT_ARM_PID_REAL[1]);
        pidController.setD(PIVOT_ARM_PID_REAL[2]);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        inputs.angleRads = encoder.getPosition();
        inputs.angVelocityRadsPerSec = encoder.getVelocity();
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
        return Math.abs(encoder.getPosition() - setpoint) < PIVOT_ARM_PID_TOLERANCE;
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

}
