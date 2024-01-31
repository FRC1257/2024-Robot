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
        public boolean breakBeamSensor = false;
    }

    /** updates inputs from robot */
    public default void updateInputs(GroundIntakeIOInputs inputs) {}
    /** sets voltage to run motor if necessary */
    public default void setVoltage(double voltage) {}
    /** sets brake mode */
    public default void setBrake(boolean brake) {
    }
}
