//this is shooterIO.java
package frc.robot.subsystems.shooterMotor;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        //inputs for the logger
        public double speed = 0.0;
        public boolean gamePiece = false;
        public double encoderPosition = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};

    }

public double RIGHTMOTOR_MAX_SPEED = -1000;
public double RIGHTMOTOR_MIN_SPEED = 1000;

public double LEFTMOTOR_MAX_SPEED = 1000;
public double LEFTMOTOR_MIN_SPEED = -1000;

    public default void updateInputs() {}
    public default void setVoltage(double shooterVolts) {} 
    public default void setBrake(boolean brake) {}
    public default void setRPIDConstants(double p, double i, double d, double ff);
    public default void setLPIDConstants(double p, double i, double d, double ff);  

}