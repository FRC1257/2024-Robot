package frc.robot.subsystems.trapPivot;

import org.littletonrobotics.junction.AutoLog;

public interface TrapPivotIO {
    // Inputs keep track of the state of the arm
    @AutoLog
    public static class TrapPivotIOInputs {
        public double angleRads = 0.0; // Arm angle
        public double angVelocityRadsPerSec = 0.0; // Angular velocity of arm
        public double appliedVolts = 0.0; // Volts applied to arm
        public double[] currentAmps = new double[] {}; // Amp measurements of pivot motors
        public double[] tempCelsius = new double[] {}; // Temperatures of pivot motors
        public double setpointAngleRads = 0.0; // Current target setpoint
    }

    public default void updateInputs(TrapPivotIOInputs inputs) {} // Runs periodically

    // Motor functions
    public default void setVoltage(double volts) {} // Sets motor voltage

    public default double getAngleRads() { return 0; } // Gets angle of motor in radians

    public default void setBrake(boolean brake) {} // Sets motor to brake if true, coast if false

    public default boolean isBrake() { return false; }; // Returns true if motor idle mode is brake, false if coast

    // PID functions
    public default void goToSetpoint(double setpoint) {} // Goes to a setpoint using PID

    public default boolean atSetpoint() { return false; } // Returns true if arm is close enough to setpoint

    public default double getP() { return 0; } // Returns P constant

    public default double getI() { return 0; } // Returns I constant

    public default double getD() { return 0; } // Returns D constant

    public default void setP(double p) {} // Sets P constant

    public default void setI(double p) {} // Sets I constant

    public default void setD(double p) {} // Sets D constant
}
