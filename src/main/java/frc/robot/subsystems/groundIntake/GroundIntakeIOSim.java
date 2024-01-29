package frc.robot.subsystems.groundIntake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeIOSim implements GroundIntakeIO {
    private PWMSparkMax groundIntakeMotor;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0.0;

    private DigitalInput breakBeamSensor;

    public GroundIntakeIOSim() {
        /** ID needs to be assigned from constants */
        groundIntakeMotor = new PWMSparkMax(0, PWMSparkMax.MotorType.kBrushless);
        groundIntakeMotor.restoreFactoryDefaults();
        groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        groundIntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        
        pidController = groundIntakeMotor.getPIDController();
        pidController.setOutputRange(-1, 1);

        encoder = groundIntakeMotor.getEncoder();
        /** Needs constants */
        encoder.setPositionConversionFactor(CONSTANT);
        encoder.setVelocityConversionFactor(CONSTANT / 60.0);
        encoder.setPosition(0.6);
        /** Encoder channel should be assigned from constants */
        absoluteEncoder = new DutyCycleEncoder(0);
        /** 
         * Unsure if these calculations are accurate
         * since I don't know the encoder's specs.
         */
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);
        /** May need recalculation */
        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);

        /** ID needs to be assigned from constants */
        breakBeamSensor = new DigitalInput(0);
    }
    
    /** Min/max angles for extension **ADD VALUES** */
    public double GROUND_INTAKE_MIN_ANGLE = 0.0;
    public double GROUND_INTAKE_MAX_ANGLE = 0.0;
    /** updates inputs from robot */
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.appliedVoltage = groundIntakeMotor.getAppliedOutput() * groundIntakeMotor.getBusVoltage();
        inputs.currentAmps = new double[] {groundIntakeMotor.getOutputCurrent()};
        inputs.tempCelcius = new double[] {groundIntakeMotor.getMotorTemperature()};
        /** This line may be unnecessary */
        inputs.encoderPosition = absoluteEncoder.getAbsolutePosition();
        inputs.angle = encoder.getPosition();
        inputs.angleRadPerSec = encoder.getVelocity();
    }
    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        groundIntakeMotor.setVoltage(voltage);
    }
    /** returns true if it is at setpoint, needs constants */
    @Override
    public boolean atSetpoint() {
        return Math.abs(absoluteEncoder.getAbsolutePosition() - setpoint) < groundIntakeMotor.GROUND_MOTOR_PID_TOLERANCE;
    }
    /** Gets break beam state */
    @Override
    public void breakBeamSensor() {
        SmartDashboard.putBoolean("Ground Intake Break Beam", breakBeamSensor.get());
    }
    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
    }
    @Override
    public void setBrake(boolean brake) {
        groundIntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
    /** starts PID, may need revision or may be redundant */
    @Override
    public void runPID(double setpoint) {

    }
    /** sets all PID targets at once */
    @Override
    public void setPIDConstants(double P, double I, double D, double FF) {
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(FF);
    }
    /** various PID get and set commands */ 
    @Override
    public void setP(double P) {
        pidController.setP(P);
    }
    @Override
    public void setI(double I) {
        pidController.setI(I);
    }
    @Override
    public void setD(double D) {
        pidController.setD(D);
    }
    @Override
    public void setFF(double FF) {
        pidController.setFF(FF);
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
