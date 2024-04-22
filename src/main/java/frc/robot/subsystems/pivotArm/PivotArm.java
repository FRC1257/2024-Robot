package frc.robot.subsystems.pivotArm;

import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.pivotArm.PivotArmConstants.*;
import frc.robot.util.misc.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.pivotArm.PivotArmConstants.PIVOT_ARM_PID;
import static frc.robot.subsystems.pivotArm.PivotArmConstants.PIVOT_ARM_FEED_FORWARD;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PivotArm extends SubsystemBase {
    private final PivotArmIOInputsAutoLogged inputs = new PivotArmIOInputsAutoLogged();

    private static final LoggedTunableNumber logP = new LoggedTunableNumber("PivotArm/P", PIVOT_ARM_PID.kP());
    private static final LoggedTunableNumber logI = new LoggedTunableNumber("PivotArm/I", PIVOT_ARM_PID.kI());
    private static final LoggedTunableNumber logD = new LoggedTunableNumber("PivotArm/D", PIVOT_ARM_PID.kD());
    private static final LoggedTunableNumber logFF = new LoggedTunableNumber("PivotArm/FF", PIVOT_ARM_PID.kFF());

    private static final LoggedTunableNumber logkS = new LoggedTunableNumber("PivotArm/kS", PIVOT_ARM_FEED_FORWARD.kS());
    private static final LoggedTunableNumber logkG = new LoggedTunableNumber("PivotArm/kG", PIVOT_ARM_FEED_FORWARD.kG());
    private static final LoggedTunableNumber logkV = new LoggedTunableNumber("PivotArm/kV", PIVOT_ARM_FEED_FORWARD.kV());
    private static final LoggedTunableNumber logkA = new LoggedTunableNumber("PivotArm/kA", PIVOT_ARM_FEED_FORWARD.kA());
    

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));


    private double setpoint = 0;

    private final PivotArmIO io;

    // Create a Mechanism2d visualization of the arm
    private MechanismLigament2d armMechanism = getArmMechanism();

    private SysIdRoutine SysId;

    public PivotArm(PivotArmIO io) {
        this.io = io;
       
        SmartDashboard.putData(getName(), this);
    
        SysId = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(PivotArmConstants.RAMP_RATE), Volts.of(PivotArmConstants.STEP_VOLTAGE), null),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), 
                (sysidLog) -> {
                    sysidLog.motor("pivot")
                    .voltage(
                        m_appliedVoltage.mut_replace(inputs.appliedVolts, Volts))
                    .angularPosition(m_angle.mut_replace(inputs.angleRads, Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(inputs.angVelocityRadsPerSec, RotationsPerSecond));
                        
                }, 
                this));
        
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PivotArm", inputs);

        armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));
        
        // Update the PID constants if they have changed
        LoggedTunableNumber.ifChanged(
            hashCode(),  () -> io.setPID(logP.get(), logI.get(), logD.get()), logP, logI, logD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setFeed(logkS.get(), logkV.get(), logkG.get(), logkA.get()), logkS, logkV, logkG, logkA);        
        // Log Inputs
        Logger.processInputs("PivotArm", inputs);

        Logger.recordOutput("PivotArm/PivotAbsoluteEncoderConnected", inputs.angleRads != PivotArmConstants.PIVOT_ARM_OFFSET);
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    @AutoLogOutput(key = "PivotArm/Close")
    public boolean isVoltageClose(double setVoltage) {
        double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
        return voltageDifference <= PivotArmConstants.PIVOT_ARM_TOLERANCE;
    }

    public void setVoltage(double motorVolts) {
        // limit the arm if its past the limit
        if (io.getAngle() > PivotArmConstants.PIVOT_ARM_MAX_ANGLE && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getAngle() < PivotArmConstants.PIVOT_ARM_MIN_ANGLE && motorVolts < 0) {
            motorVolts = 0;
        }

        if(SmartDashboard.getBoolean("Turbo Mode", false)){
            io.setVoltage(0);
        } else {
            io.setVoltage(motorVolts);
        }
        isVoltageClose(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void holdPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
        Logger.recordOutput("PivotArm/Setpoint", setpoint);
    }

    public void addPID(double setpointAdd) {
        this.setpoint += setpointAdd;
        this.setpoint = MathUtil.clamp(this.setpoint, PivotArmConstants.PIVOT_ARM_MIN_ANGLE, PivotArmConstants.PIVOT_ARM_MAX_ANGLE);
        
        Logger.recordOutput("PivotArm/Setpoint", setpoint);
    }

    public boolean atSetpoint() {
        return Math.abs(io.getAngle() - setpoint) < PivotArmConstants.PIVOT_ARM_PID_TOLERANCE && Math.abs(getVelocity()) < PivotArmConstants.PIVOT_ARM_PID_VELOCITY_TOLERANCE;
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(inputs.angleRads);
    }

    public double getVelocity() {
        return inputs.angVelocityRadsPerSec;
    }

    public Rotation2d getSetpoint() {
        return new Rotation2d(setpoint);
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return armMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", 0.4, 0, 5, new Color8Bit(Color.kAqua));
    }

    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }

    public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
        return new FunctionalCommand(
            () -> setPID(setpointSupplier.getAsDouble()), 
            () -> {
                setPID(setpointSupplier.getAsDouble());
                runPID();
            }, 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDCommandForever(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDHoldCommand() {
        return new FunctionalCommand(
            () -> setPID(getAngle().getRadians()), 
            () -> holdPID(), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDCommand(DoubleSupplier setpointSupplier) {
        return new FunctionalCommand(
            () -> setPID(setpointSupplier.getAsDouble()), 
            () -> {
                Logger.recordOutput("PivotArmSpeakerAngle", setpointSupplier.getAsDouble());
                setPID(setpointSupplier.getAsDouble());
                runPID();
            }, 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }

    // Allows manual control of the pivot arm for PID tuning
    public Command ManualCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> move(speedSupplier.getAsDouble()), 
            () -> move(speedSupplier.getAsDouble()), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command stop() {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(0),
            (stop) -> io.stop(),
            () -> false,
            this
        );
      }
    //no commmand yalee

    public Command bringDownCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                move(-1);
                setpoint = 0;
            }, 
            (interrupted) -> {
                move(0);
            }, 
            () -> {
                return io.getAngle() < 0.1;
            }, 
            this);
    }

    public Command quasistaticForward() {
        return SysId
            .quasistatic(Direction.kForward)
            .until(() -> getAngle().getRadians() > PivotArmConstants.PIVOT_ARM_MAX_ANGLE)
                .alongWith(new InstantCommand(() -> Logger.recordOutput("PivotArm/sysid-test-state-", "quasistatic-forward")));
      }
    
    public Command quasistaticBack() {
        return SysId
            .quasistatic(Direction.kReverse)
            .until(() -> getAngle().getRadians() < PivotArmConstants.PIVOT_ARM_MIN_ANGLE)
                .alongWith(new InstantCommand(() -> Logger.recordOutput("PivotArm/sysid-test-state-", "quasistatic-reverse")));
    }

    public Command dynamicForward() {
        return SysId
            .dynamic(Direction.kForward)
            .until(() -> getAngle().getRadians() > PivotArmConstants.PIVOT_ARM_MAX_ANGLE)
                .alongWith(new InstantCommand(() -> Logger.recordOutput("PivotArm/sysid-test-state-", "dynamic-forward")));
    }

    public Command dynamicBack() {
        return SysId
            .dynamic(Direction.kReverse)
            .until(() -> getAngle().getRadians() < PivotArmConstants.PIVOT_ARM_MIN_ANGLE)
                .alongWith(new InstantCommand(() -> Logger.recordOutput("PivotArm/sysid-test-state-", "dynamic-reverse")));
    }

}