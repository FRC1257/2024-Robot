package frc.robot.subsystems.groundIntake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeIOSim implements GroundIntakeIO {
    private PWMSparkMax groundIntakeMotor;
  
    private DigitalInput breakBeamSensor;

    public GroundIntakeIOSim() {
        /** ID needs to be assigned from constants */
        groundIntakeMotor = new PWMSparkMax(0, PWMSparkMax.MotorType.kBrushless);
        groundIntakeMotor.restoreFactoryDefaults();
        groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        groundIntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
       
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
    }
    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        groundIntakeMotor.setVoltage(voltage);
    }
   
    /** Gets break beam state */
    @Override
    public void breakBeamSensor() {
        SmartDashboard.putBoolean("Ground Intake Break Beam", breakBeamSensor.get());
    }
 
    @Override
    public void setBrake(boolean brake) {
        groundIntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
