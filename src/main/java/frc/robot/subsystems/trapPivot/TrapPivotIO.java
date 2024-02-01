package frc.robot.subsystems.trapPivot;

import org.littletonrobotics.junction.AutoLog;

public interface TrapPivotIO {
    // Inputs 
    @AutoLog
    public static class TrapPivotIOInputs {
        public double angleRads = 0.0; // Arm angle
        public double angVelocityRadsPerSec = 0.0; // Angular velocity of arm
        public double appliedVolts = 0.0; // Volts applied to arm
        public double[] currentAmps = new double[] {}; // Amps of pivot motors
        public double[] tempCelsius = new double[] {}; // Temperatures of pivot motors
    }

    public default void updateInputs() {} // Runs periodically and updates inputs

    // Motor functions
    public default void setVoltage() {} // Sets motor voltage

    public default void getAngle() {} // Returns angle of arm

    public default void getAngVelocity() {} // Returns angular velocity of arm

    // PID functions
    public default void goToSetpoint(double setpoint) {} // Goes to a setpoint using PID

    public default double getP() { return 0; } // Returns P constant

    public default double getI() { return 0; } // Returns I constant

    public default double getD() { return 0; } // Returns D constant

    public default void setP(double p) {} // Sets P constant

    public default void setI(double p) {} // Sets I constant

    public default void setD(double p) {} // Sets D constant
}
