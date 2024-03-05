package frc.robot.subsystems.pivotArm;

import static frc.robot.Constants.ElectricalLayout.LEFT_SLAVE_ID;
import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_ID;
import static frc.robot.Constants.ElectricalLayout.RIGHT_SLAVE_BACK_ID;
import static frc.robot.Constants.ElectricalLayout.RIGHT_SLAVE_FRONT_ID;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

public class PivotArmIOSparkMax implements PivotArmIO {
    // Motor and Encoders
    private CANSparkMax pivotMotor, leftSlave, rightSlaveFront, rightSlaveBack;
    private SparkPIDController pidController;
    
    private DutyCycleEncoder absoluteEncoder;
    private RelativeEncoder motorEncoder;

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
        //I swear to go these inverts might have to be after the following
        //pivotMotor.setInverted(false);
        //leftSlave.setInverted(false);
        //rightSlaveFront.setInverted(true);
        //rightSlaveBack.setInverted(true);
        //can't invert like this
        setBrake(true);

        leftSlave.follow(pivotMotor, false);
        rightSlaveFront.follow(pivotMotor, true);
        rightSlaveBack.follow(pivotMotor, true);

        setBrake(true);
        
        pivotMotor.enableVoltageCompensation(12.0);

        pivotMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        leftSlave.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightSlaveFront.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightSlaveBack.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        pivotMotor.burnFlash();
        leftSlave.burnFlash();
        rightSlaveFront.burnFlash();
        rightSlaveBack.burnFlash();

        //wasn't burning the flash to all the motors, this might be the issue

        configurePID();

        /* 
        absoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR / 60.0); */

        absoluteEncoder = new DutyCycleEncoder(ElectricalLayout.ABSOLUTE_ENCODER_ID);
        absoluteEncoder.setDistancePerRotation(2 * Constants.PI * PivotArmConstants.POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setDutyCycleRange(1/1024.0, 1023.0/1024.0);
        // absoluteEncoder.reset();

        //0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

        motorEncoder = pivotMotor.getEncoder();
        motorEncoder.setPositionConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR);
        motorEncoder.setVelocityConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR / 60.0);
    }

    private void configurePID() {
        pidController = pivotMotor.getPIDController();
        pidController.setOutputRange(PivotArmConstants.PIVOT_ARM_MIN_ANGLE, PivotArmConstants.PIVOT_ARM_MAX_ANGLE);
        pidController.setP(PivotArmConstants.PIVOT_ARM_PID_REAL[0]);
        pidController.setI(PivotArmConstants.PIVOT_ARM_PID_REAL[1]);
        pidController.setD(PivotArmConstants.PIVOT_ARM_PID_REAL[2]);
        pidController.setFF(PivotArmConstants.PIVOT_ARM_PID_REAL[3]);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        inputs.angleRads = absoluteEncoder.getAbsolutePosition();
        Logger.recordOutput("PivotOtgerThing", absoluteEncoder.getDistance());
        inputs.angVelocityRadsPerSec = motorEncoder.getVelocity();
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
        return absoluteEncoder.getAbsolutePosition();
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
        leftSlave.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightSlaveFront.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightSlaveBack.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpoint) < PivotArmConstants.PIVOT_ARM_PID_TOLERANCE;
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
