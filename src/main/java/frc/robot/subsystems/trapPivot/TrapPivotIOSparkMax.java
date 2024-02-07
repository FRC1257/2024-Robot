package frc.robot.subsystems.trapPivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.TrapPivot.*;
import static frc.robot.Constants.TrapPivot.TrapPivotReal.*;

public class TrapPivotIOSparkMax implements TrapPivotIO {
    // Standard WPILib classes for motor stuff
    // A motor has an encoder which tracks position/velocity
    // It also has a PID controller, which gets you to a setpoint as quickly and smoothly as possible
    private CANSparkMax pivotMotor;
    private SparkPIDController pidController;
    private SparkAbsoluteEncoder encoder;
    
    // Current PID setpoint
    private double setpoint = 0;

    public TrapPivotIOSparkMax() {
        // Instantiates and configures motor
        pivotMotor = new CANSparkMax(TRAP_PIVOT_ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setInverted(false);
        pivotMotor.enableVoltageCompensation(12.0);
        pivotMotor.setSmartCurrentLimit(30);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.burnFlash();

        // Configure encoders and PID
        configureEncoders();
        configurePID();
    }

    // Converts encoder position and velocity units
    // Default encoder units are rotations and RPM
    private void configureEncoders() {
        encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        encoder.setPositionConversionFactor(Math.PI * 2 / TRAP_PIVOT_REDUCTION_REAL); // Converts position unit from rotations to radians
        encoder.setVelocityConversionFactor(Math.PI * 2 / TRAP_PIVOT_REDUCTION_REAL / 60.0); // Converts velocity unit from RPM to rads/sec
    }

    // Sets P, I, and D constants
    private void configurePID() {
        pidController = pivotMotor.getPIDController();
        pidController.setP(TRAP_PIVOT_PID_REAL[0]); // P (proportional) term changes motor speed based on distance away from setpoint
        pidController.setI(TRAP_PIVOT_PID_REAL[1]); // I (integral) term prevents overshooting and undershooting
        pidController.setD(TRAP_PIVOT_PID_REAL[2]); // D (derivative) term prevents oscillation and unstable speed
    }

    // Runs periodically and updates inputs based on the state of the arm
    @Override
    public void updateInputs(TrapPivotIOInputs inputs) {
        inputs.angleRads = encoder.getPosition();
        inputs.angVelocityRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = new double[] { pivotMotor.getOutputCurrent() };
        inputs.tempCelsius = new double[] { pivotMotor.getMotorTemperature() };
    }

    // Sets voltage of motor
    @Override
    public void setVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    // Returns current arm position
    @Override
    public double getAngleRads() {
        return encoder.getPosition();
    }

    // Returns true if idle mode is brake, false if coast
    @Override
    public boolean isBrake() {
        return pivotMotor.getIdleMode() == IdleMode.kBrake;
    }

    // Sets motor to brake or coast when idle
    @Override
    public void setBrake(boolean brake) {
        pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    // Calculate motor speed to go to setpoint
    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(this.setpoint, ControlType.kPosition);
    }

    // Checks if you are close enough to the setpoint
    @Override
    public boolean atSetpoint() {
        return Math.abs(getAngleRads() - setpoint) < TRAP_PIVOT_PID_TOLERANCE;
    }

    // Return P, I, and D constants
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

    // Set P, I, and D constants
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
