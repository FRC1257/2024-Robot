package frc.robot.subsystems.groundIntake;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        /** 
         * Some of these may be unnecessary if no
         * NEOs are used.
         */
        public double appliedVoltage = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
        public double encoderPosition = 0.0;
        public double angle = 0.0;
        public double angleRadPerSec = 0.0;
    }

    /** Min/max angles for extension **ADD VALUES** */
    public double GROUND_INTAKE_MIN_ANGLE = 0.0;
    public double GROUND_INTAKE_MAX_ANGLE = 0.0;
    /** updates inputs from robot */
    public default void updateInputs(GroundIntakeIOInputs inputs) {}
    /** sets voltage to run motor if necessary */
    public default void setVoltage(double voltage) {}
    /** returns true if ground intake is at setpoint */
    public default boolean atSetpoint() {
        return false;
    }
    /** Outputs to shuffleboard. It's here due to the sensor */
    public default void breakBeamSensor() {}
    /** sets ground intake to setpoint */
    public default void goToSetpoint(double setpoint) {
    }
    /** sets brake mode */
    public default void setBrake(boolean brake) {
    }
    /** starts PID */
    public default void runPID(double setpoint) {}
    /** sets all PID targets */
    public default void setPIDConstants(double P, double I, double D, double FF) {}
    /** various PID get and set commands */ 
    public default void setP(double P) {}

    public default void setI(double I) {}

    public default void setD(double D) {}

    public default void setFF(double FF) {}

    public default double getP() { return 0.0; }

    public default double getI() { return 0.0; }

    public default double getD() { return 0.0; }

    public default double getFF() { return 0.0; }
}
