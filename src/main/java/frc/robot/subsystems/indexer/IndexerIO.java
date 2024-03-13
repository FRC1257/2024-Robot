package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

/**
 * The IntakeIO interface represents the input/output interface for the intake subsystem.
 * It provides methods to update inputs from the robot, control the intake motor, and retrieve
 * PID constants for velocity control.
 */
public interface IndexerIO {
    /**
     * The IntakeIOInputs class represents the input values for the intake subsystem.
     * It contains fields for velocity, applied voltage, speed setpoint, break beam status,
     * current amps, and temperature in Celsius.
     */
    @AutoLog
    public static class IndexerIOInputs {
        /** 
         * The velocity of the intake motor in radians per second.
         */
        public double velocityRadsPerSec = 0.0;
        
        /**
         * The applied voltage to run the intake motor.
         */
        public double appliedVoltage = 0.0;
        
        /**
         * The speed setpoint for velocity control.
         */
        public double speedSetpoint = 0.0;
        
        /**
         * The status of the break beam sensor.
         */
        public boolean breakBeam = false;
        
        /**
         * The array of current values in amps for each motor.
         */
        public double[] currentAmps = new double[] {};
        
        /**
         * The array of temperature values in Celsius for each motor.
         */
        public double[] tempCelcius = new double[] {};
    }

    /**
     * Updates the input values from the robot.
     * 
     * @param inputs the IntakeIOInputs object containing the updated input values
     */
    public default void updateInputs(IndexerIOInputs inputs) {}

    /**
     * Sets the voltage to run the intake motor if necessary.
     * 
     * @param voltage the voltage to be applied to the motor
     */
    public default void setVoltage(double voltage) {}

    /**
     * Sets the PID constants for velocity control.
     * 
     * @param p the proportional constant
     * @param i the integral constant
     * @param d the derivative constant
     */
    public default void setPIDConstants(double p, double i, double d) {}
    
    /**
     * Stops the intake motor by setting the voltage to 0.
     */
    public default void stop() {
        setVoltage(0.0);
    }

    /**
     * Sets the brake mode for the intake motor.
     * 
     * @param brake true to enable brake mode, false otherwise
     */
    public default void setBrake(boolean brake) {}

    /**
     * Checks if an object is currently being intaked.
     * 
     * @return true if an object is being intaked, false otherwise
     */
    public default boolean isIntaked() {return true;}

    /**
     * Sets the speed of the intake motor.
     * 
     * @param speed the speed of the intake motor
     */
    public default void setSpeed(double speed) {}

    /**
     * Sets the proportional constant for velocity control.
     * 
     * @param p the proportional constant
     */
    public default void setP(double p) {}
    
    /**
     * Sets the integral constant for velocity control.
     * 
     * @param i the integral constant
     */
    public default void setI(double i) {}

    /**
     * Sets the derivative constant for velocity control.
     * 
     * @param d the derivative constant
     */
    public default void setD(double d) {}

    /**
     * Retrieves the proportional constant for velocity control.
     * 
     * @return the proportional constant
     */
    public default double getP() { return 0.0; }

    /**
     * Retrieves the integral constant for velocity control.
     * 
     * @return the integral constant
     */
    public default double getI() { return 0.0; }

    /**
     * Retrieves the derivative constant for velocity control.
     * 
     * @return the derivative constant
     */
    public default double getD() { return 0.0; }
}
