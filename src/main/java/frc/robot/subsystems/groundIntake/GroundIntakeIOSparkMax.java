package frc.robot.subsystems.groundIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.GroundIntake.GroundIntakeSimConstants.*;

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

    private CANSparkMax GroundIntakeMotor;
    private RelativeEncoder GroundintakeEncoder;
    private SparkPIDController velocityPID;

    private double desiredSpeed;

    public GroundIntakeIOSparkMax() {
        /** ID needs to be assigned from constants */
        setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
        GroundIntakeMotor = new CANSparkMax(ElectricalLayout.GROUND_INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        GroundIntakeMotor.restoreFactoryDefaults();
        GroundIntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        GroundIntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        GroundintakeEncoder = GroundIntakeMotor.getEncoder();


        velocityPID = GroundIntakeMotor.getPIDController();
    }

    /** updates inputs from robot */
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.appliedVoltage = GroundIntakeMotor.getAppliedOutput() * GroundIntakeMotor.getBusVoltage();
        inputs.currentAmps = new double[] { GroundIntakeMotor.getOutputCurrent() };
        inputs.tempCelcius = new double[] { GroundIntakeMotor.getMotorTemperature() };
        inputs.velocityRadsPerSec = GroundintakeEncoder.getVelocity();
        inputs.speedSetpoint = desiredSpeed;
    }

    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        GroundIntakeMotor.setVoltage(voltage);
    }

    /** sets brake mode to stop */
    @Override
    public void setBrake(boolean brake) {
        GroundIntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }


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
