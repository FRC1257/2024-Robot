package frc.robot.subsystems.intake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.IntakePhysicalConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElectricalLayout;

/** Need to import Constants files/classes */
// 

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController velocityPID;

    private DigitalInput breakBeam;

    private double desiredSpeed;

    public IntakeIOSparkMax() {
        /** ID needs to be assigned from constants */
        //setPIDConstants(kIntakeP, kIntakeI, kIntakeD);
        motor = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        /** Current limit should be added to Constants.java when known */
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        encoder = motor.getEncoder();

        breakBeam = new DigitalInput(ElectricalLayout.INTAKE_BREAK_BEAM);

        velocityPID = motor.getPIDController();
    }

    /** updates inputs from robot */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] { motor.getOutputCurrent() };
        inputs.tempCelcius = new double[] { motor.getMotorTemperature() };
        inputs.velocityRadsPerSec = encoder.getVelocity();
        inputs.speedSetpoint = desiredSpeed;
        //inputs.breakBeam = breakBeam.get();
    }

    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage * 10);
    }

    /** sets brake mode to stop */
    @Override
    public void setBrake(boolean brake) {
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean isIntaked() {
        return breakBeam.get();
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
