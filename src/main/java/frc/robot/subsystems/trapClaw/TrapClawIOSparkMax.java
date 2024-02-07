package frc.robot.subsystems.trapClaw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ElectricalLayout.TRAP_CLAW_ID;
import static frc.robot.Constants.TrapClaw.*;

public class TrapClawIOSparkMax implements TrapClawIO {
    // Standard classes for motor stuff
    // Motor has an encoder, which tracks its velocity
    private CANSparkMax clawMotor;
    private RelativeEncoder encoder;

    // Configures motor and encoder
    public TrapClawIOSparkMax() {
        clawMotor = new CANSparkMax(TRAP_CLAW_ID, MotorType.kBrushless);
        clawMotor.restoreFactoryDefaults();

        clawMotor.setInverted(false);
        clawMotor.enableVoltageCompensation(12);
        clawMotor.setSmartCurrentLimit(30);
        clawMotor.burnFlash();

        // Encoder velocity is in RPM by default, you have to convert it to rads/sec
        encoder = clawMotor.getEncoder();
        encoder.setVelocityConversionFactor(Math.PI * 2 / TRAP_CLAW_REDUCTION_REAL / 60);
    }

    // Updates inputs periodically based on state of motor
    @Override
    public void updateInputs(TrapClawIOInputs inputs) {
        inputs.velocityRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = clawMotor.getAppliedOutput() * clawMotor.getBusVoltage();
        inputs.currentAmps = new double[]{ clawMotor.getOutputCurrent() };
        inputs.tempCelsius = new double[]{ clawMotor.getMotorTemperature() };
    }

    // Sets voltage of motor
    @Override
    public void setVoltage(double volts) {
        clawMotor.setVoltage(volts);
    }
}
