package frc.robot.subsystems.trapClaw;

import org.littletonrobotics.junction.AutoLog;

public interface TrapClawIO {
    @AutoLog
    public static class TrapClawIOInputs {
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = {0.0};
        public double[] tempCelsius = {0.0};
    }

    public default void updateInputs(TrapClawIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default double getVelocity() { return 0; }
}
