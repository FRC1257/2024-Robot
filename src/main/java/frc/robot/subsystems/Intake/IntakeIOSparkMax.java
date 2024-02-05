package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase; 
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/** Need to import Constants files/classes */

public class IntakeIOSparkMax implements IntakeIO {
    
    private CANSparkMax IntakeMotor;


    private boolean intook;

    public IntakeIOSparkMax() {
        /** ID needs to be assigned from constants */
        IntakeMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        IntakeMotor.restoreFactoryDefaults();
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        /** Current limit should be added to Constants.java when known */
        IntakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

 
    }
    
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
    /** sets brake mode to stop */
    @Override
    public void setBrake(boolean brake) {
        IntakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean isIntaked(){
        return intook;
    }
}
