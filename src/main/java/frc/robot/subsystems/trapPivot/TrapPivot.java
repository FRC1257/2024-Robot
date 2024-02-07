package frc.robot.subsystems.trapPivot;

import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_ANGLE_THRESHOLD;
import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_EXTEND_ANGLE_RADS;
import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_LENGTH_M;
import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_MAX_ANGLE_RADS;
import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_MIN_ANGLE_RADS;
import static frc.robot.Constants.TrapPivot.TRAP_PIVOT_RETRACT_ANGLE_RADS;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapPivot extends SubsystemBase {
    // State of the arm
    private final TrapPivotIOInputsAutoLogged inputs = new TrapPivotIOInputsAutoLogged();

    // PID constants are logged to the shuffleboard so you can change them at runtime
    private LoggedDashboardNumber logP;
    private LoggedDashboardNumber logI;
    private LoggedDashboardNumber logD;

    // Motor idle mode is also logged to shuffleboard so you can change it at runtime
    private LoggedDashboardBoolean logIsBrake;

    // Current PID setpoint
    private double setpoint = 0;

    // IO interfact that interacts with motor
    private final TrapPivotIO io;

    // Mechanism visualization of the arm
    private MechanismLigament2d armMechanism;

    public TrapPivot(TrapPivotIO io) {
        // Initializes arm mechanism and IO interface
        armMechanism = getMechanism();
        this.io = io;

        // Logs trap arm data to smart dashboard
        SmartDashboard.putData(getName(), this);

        // Logs PID constants to shuffleboard
        logP = new LoggedDashboardNumber("TrapPivot/P", io.getP());
        logI = new LoggedDashboardNumber("TrapPivot/I", io.getI());
        logD = new LoggedDashboardNumber("TrapPivot/D", io.getD());

        // Logs idle mode to shuffleboard
        logIsBrake = new LoggedDashboardBoolean("TrapPivot/Is Brake", io.isBrake());
    }

    // Runs periodically
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads)); // Changes mechanism visualization based on position

        // Update PID constants if they have changed
        if(logP.get() != io.getP())
            io.setP(logP.get());

        if(logI.get() != io.getI())
            io.setI(logI.get());

        if(logD.get() != io.getD())
            io.setD(logD.get());
        
        // Updates motor idle mode if it has changed
        if(logIsBrake.get() != io.isBrake()) {
            io.setBrake(logIsBrake.get());
        }

        // Process inputs
        Logger.processInputs(getName(), inputs);
    }

    // Sets motor voltage if arm is within limits
    public void setVoltage(double volts) {
        if(io.getAngleRads() > TRAP_PIVOT_MAX_ANGLE_RADS && volts > 0)
            volts = 0;
        else if(io.getAngleRads() < TRAP_PIVOT_MIN_ANGLE_RADS && volts < 0)
            volts = 0;
        
        io.setVoltage(volts);
    }

    // Moves arm a certain speed (-1 <= speed <= 1)
    public void move(double speed) {
        setVoltage(speed * 12);
    }

    // Sets current PID setpoint
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Runs PID calculations
    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    // Checks if the arm is close enough to the setpoint
    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    // Sets arm mechanism
    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    // Creates and returns a new arm mechanism
    public MechanismLigament2d getMechanism() {
        return new MechanismLigament2d(getName(), TRAP_PIVOT_LENGTH_M, 0, 5, new Color8Bit(Color.kAqua));
    }

    // If the arm is extended, it should retract, and vice versa
    public void toggleSetpoint() {
        if(io.getAngleRads() > TRAP_PIVOT_ANGLE_THRESHOLD)
            setSetpoint(TRAP_PIVOT_RETRACT_ANGLE_RADS);
        else
            setSetpoint(TRAP_PIVOT_EXTEND_ANGLE_RADS);
    }

    // Command that extends or retracts the arm
    public Command ExtendRetractCommand() {
        return new FunctionalCommand(
            () -> toggleSetpoint(),
            () -> runPID(),
            (stop) -> move(0),
            this::atSetpoint,
            this
        );
    }
}
