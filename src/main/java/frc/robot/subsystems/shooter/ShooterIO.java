//this is shooterIO.java
package frc.robot.subsystems.drive;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        //inputs for the logger
        public double velocityMeters = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};

    }
    public default void updateInputs() {}
    public default void setVoltage(double voltage) {}
    public default float getDistance(){return 0.0;}
}