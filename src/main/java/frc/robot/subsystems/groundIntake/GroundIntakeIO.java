package frc.robot.subsystems.groundIntake;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        
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
