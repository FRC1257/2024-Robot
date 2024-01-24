package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class GroundIntakeIOSparkMax implements GroundIntakeIO {
    
    private CANSparkMax groundIntakeMotor;
    private SparkPIDController pidController;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0.0;

    public GroundIntakeIOSparkMax() {
        groundIntakeMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        groundIntakeMotor.restoreFactoryDefaults();
        groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        groundIntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        
        pidController = groundIntakeMotor.getPIDController();
        pidController.setOutputRange(-1, 1);
        /** Encoder channel may have to be changed */
        absoluteEncoder = new DutyCycleEncoder(0);
        /** 
         * Unsure if these calculations are accurate
         * since I don't know the encoder's specs.
         */
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);
        /** Still need to initialize a break-beam sensor */
    }
    
    /** Min/max angles for extension **ADD VALUES** */
    public double GROUND_INTAKE_MIN_ANGLE = 0.0;
    public double GROUND_INTAKE_MAX_ANGLE = 0.0;
    /** updates inputs from robot */
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {

    }
    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {

    }
    /** starts PID */
    @Override
    public void runPID(double setpoint) {

    }
    /** sets PIDFF target */
    @Override
    public void setPIDFF(double FF) {

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
