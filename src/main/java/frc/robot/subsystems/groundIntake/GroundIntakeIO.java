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
    /** updates inputs from robot */
    public void updateInputs(GroundIntakeIOInputs inputs);
    /** sets voltage to run motor if necessary */
    public void setVoltage(double voltage);
    /** starts PID */
    public void runPID(double setpoint);
    /** sets PIDFF target */
    public void setPIDFF(double FF);
    /** various PID get and set commands */ 
    public void setP(double P);

    public void setI(double I);

    public void setD(double D);

    public void setFF(double FF);

    public double getP();

    public double getI();

    public double getD();

    public double getFF();
}
