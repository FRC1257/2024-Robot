package frc.robot.subsystems.trapClaw;

import org.littletonrobotics.junction.AutoLog;

public interface TrapClawIO {
    @AutoLog
    public static class TrapClawIOInputs {
        public double angleRads;
        public double angVelocityRadsPerSec;
        public double appliedVolts;
        public double[] currentAmps;
        public double[] tempCelsius;
    }
}
