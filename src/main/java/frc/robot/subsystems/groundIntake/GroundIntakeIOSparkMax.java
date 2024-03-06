package frc.robot.subsystems.groundIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.subsystems.groundIntake.GroundIntakeConstants.GroundIntakePhysicalConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElectricalLayout;

/** Need to import Constants files/classes */
// 

public class GroundIntakeIOSparkMax implements GroundIntakeIO {

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController velocityPID;

    private double desiredSpeed;

    public GroundIntakeIOSparkMax() {
        /** ID needs to be assigned from constants */
        //setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
        motor = new CANSparkMax(ElectricalLayout.GROUND_INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        encoder = motor.getEncoder();

        velocityPID = motor.getPIDController();
    }

    /** updates inputs from robot */
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] { motor.getOutputCurrent() };
        inputs.tempCelcius = new double[] { motor.getMotorTemperature() };
        inputs.velocityRadsPerSec = encoder.getVelocity();
        inputs.speedSetpoint = desiredSpeed;
    }

    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /** sets brake mode to stop */
    @Override
    public void setBrake(boolean brake) {
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /** sets speed of motor */
    @Override
    public void setSpeed(double speed) {
        desiredSpeed = speed;
        velocityPID.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void setP(double p) {
        velocityPID.setP(p);
    }

    @Override
    public void setI(double i) {
        velocityPID.setI(i);
    }

    @Override
    public void setD(double d) {
        velocityPID.setD(d);
    }

    @Override
    public double getP() {
        return velocityPID.getP();
    }

    @Override
    public double getI() {
        return velocityPID.getI();
    }

    @Override
    public double getD() {
        return velocityPID.getD();
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setPIDConstants(double p, double i, double d) {
        velocityPID.setP(p);
        velocityPID.setI(i);
        velocityPID.setD(d);
    }
}
