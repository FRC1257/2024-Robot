package frc.robot.subsystems.pivotArm;

import static frc.robot.Constants.PivotArm.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

public class PivotArmIOSparkMax implements PivotArmIO {
    // Motor and Encoders
    private CANSparkMax pivotMotor, leftSlave, rightSlaveFront, rightSlaveBack;
    private SparkPIDController pidController;
    
    private AbsoluteEncoder absoluteEncoder;

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

        absoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        
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
        inputs.angleRads = absoluteEncoder.getPosition();
        inputs.angVelocityRadsPerSec = absoluteEncoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
        inputs.setpointAngleRads = setpoint;
    }

    /** Run open loop at the specified voltage. */
    @Override
    public void setVoltage(double motorVolts) {
        pivotMotor.setVoltage(motorVolts);
    }

    /** Returns the current distance measurement. */
    @Override
    public double getAngle() {
        return absoluteEncoder.getPosition();
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
        return Math.abs(absoluteEncoder.getPosition() - setpoint) < PIVOT_ARM_PID_TOLERANCE;
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
