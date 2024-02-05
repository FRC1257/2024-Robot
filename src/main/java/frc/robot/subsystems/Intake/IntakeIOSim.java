package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.CANSparkMax;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSim implements IntakeIO {
    //replace with DCMOTORSIM
    private CANSparkMax IntakeMotor;
  

    private boolean intook;

    public IntakeIOSim() {
        /** ID needs to be assigned from constants */
        IntakeMotor = new PWMSparkMax(0, PWMSparkMax.MotorType.kBrushless);
        IntakeMotor.restoreFactoryDefaults();
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        IntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
       
        /** ID needs to be assigned from constants */
        breakBeamSensor = new DigitalInput(0);
    }
    
    /** Min/max angles for extension **ADD VALUES** */
    public double INTAKE_MIN_ANGLE = 0.0;
    public double INTAKE_MAX_ANGLE = 0.0;
    /** updates inputs from robot */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = IntakeMotor.getAppliedOutput() * IntakeMotor.getBusVoltage();
        inputs.currentAmps = new double[] {IntakeMotor.getOutputCurrent()};
        inputs.tempCelcius = new double[] {IntakeMotor.getMotorTemperature()};
    }
    /** sets voltage to run motor if necessary */
    @Override
    public void setVoltage(double voltage) {
        IntakeMotor.setVoltage(voltage);
    }

    @Override
    public void setBrake(boolean brake) {
        IntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
    
    @Override
    public boolean isIntaked(){
        return intook;
    }
}
