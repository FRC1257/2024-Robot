package frc.robot.subsystems.trapClaw;

import org.littletonrobotics.junction.AutoLog;

public interface TrapClawIO {
    // Inputs describe the current state of the claw  
    @AutoLog
    public static class TrapClawIOInputs {
        public double velocityRadsPerSec = 0.0; // Velocity of flywheel
        public double appliedVolts = 0.0; // Volts applied to motor
        public double[] currentAmps = {0.0}; // Amp measurements of motors
        public double[] tempCelsius = {0.0}; // Temperatures of motors
    }

    // Runs periodically and updates inputs to match the current state
    public default void updateInputs(TrapClawIOInputs inputs) {}

    // Sets voltage of motor
    public default void setVoltage(double volts) {}
}
