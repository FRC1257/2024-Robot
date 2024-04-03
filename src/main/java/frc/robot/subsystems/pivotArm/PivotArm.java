package frc.robot.subsystems.pivotArm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

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

    private LoggedDashboardNumber logP;
    private LoggedDashboardNumber logI;
    private LoggedDashboardNumber logD;
    private LoggedDashboardNumber logFF;

    private LoggedDashboardNumber logkS;
    private LoggedDashboardNumber logkG;
    private LoggedDashboardNumber logkV;
    private LoggedDashboardNumber logkA;

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
    
        logP = new LoggedDashboardNumber("PivotArm/P", io.getP());
        logI = new LoggedDashboardNumber("PivotArm/I", io.getI());
        logD = new LoggedDashboardNumber("PivotArm/D", io.getD());
        logFF = new LoggedDashboardNumber("PivotArm/FF", io.getFF());

        logkS = new LoggedDashboardNumber("PivotArm/kS", io.getkS());
        logkG = new LoggedDashboardNumber("PivotArm/kG", io.getkG());
        logkV = new LoggedDashboardNumber("PivotArm/kV", io.getkV());
        logkA = new LoggedDashboardNumber("PivotArm/kG", io.getkA());

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
        if (logP.get() != io.getP()) 
            io.setP(logP.get());
        
        if (logI.get() != io.getI())
            io.setI(logI.get());
        
        if (logD.get() != io.getD())
            io.setD(logD.get());
        
        if (logFF.get() != io.getFF())
            io.setFF(logFF.get()); 

        if (logkS.get() != io.getkS())
            io.setkS(logkS.get());

        if (logkG.get() != io.getkG())
            io.setkG(logkG.get());   

        if (logkV.get() != io.getkV())
            io.setkV(logkV.get());

        if (logkA.get() != io.getkA())
            io.setkG(logkA.get());   
        
        // Log Inputs
        Logger.processInputs("PivotArm", inputs);
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
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
    }

    public void move(double speed) {
        setVoltage(speed);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void holdPID() {
        io.holdSetpoint(setpoint);
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

