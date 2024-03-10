package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class IntakeSubsystem extends SubsystemBase {

        private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController velocityPID;

    private double desiredSpeed;


    public IntakeSubsystem () {
        SmartDashboard.putData(getName(), this);
                /** ID needs to be assigned from constants */
        //setPIDConstants(kIntakeP, kIntakeI, kIntakeD);
        motor = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        /** Current limit should be added to Constants.java when known */
        motor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        encoder = motor.getEncoder();

        velocityPID = motor.getPIDController();

    }

    public void periodic() {
        
    }

    /**
     * Uses input from controller to set speed of the flywheel
     * and is used as the default command for the ground intake
    */
    public Command speedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> setSpeed(speed.getAsDouble()),
            (stop) -> stop(),
            () -> false,
            this
        );
    }
    // Allows manual command of the flywheel for testing
    public Command manualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(voltage.getAsDouble()),
            (stop) -> stop(),
            () -> false,
            this
        );
    }

    public Command manualCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(voltage),
            (stop) -> stop(),
            () -> false,
            this
        );
    }

    public Command stopCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> setVoltage(0),
            (stop) -> stop(),
            () -> false,
            this
        );
      }

    /** sets voltage to run motor if necessary */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage * 10);
    }

    /** sets brake mode to stop */
    public void setBrake(boolean brake) {
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        desiredSpeed = speed;
        velocityPID.setReference(speed, ControlType.kVelocity);
    }

    public void setP(double p) {
        velocityPID.setP(p);
    }

    public void setI(double i) {
        velocityPID.setI(i);
    }

    public void setD(double d) {
        velocityPID.setD(d);
    }

    public double getP() {
        return velocityPID.getP();
    }

    public double getI() {
        return velocityPID.getI();
    }

    public double getD() {
        return velocityPID.getD();
    }

    public void setPIDConstants(double p, double i, double d) {
        velocityPID.setP(p);
        velocityPID.setI(i);
        velocityPID.setD(d);
    }

    public void stop() {
        setVoltage(0.0);
    }
}
