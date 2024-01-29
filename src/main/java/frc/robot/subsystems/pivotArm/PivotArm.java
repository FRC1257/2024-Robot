package frc.robot.subsystems.pivotArm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

import static frc.robot.Constants.PivotArm.*;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_PID;

public class PivotArm extends SubsystemBase {
    private final PivotArmIOInputsAutoLogged inputs = new PivotArmIOInputsAutoLogged();

    public enum State {
        MANUAL,
        PID
    }

    private LoggedDashboardNumber p = new LoggedDashboardNumber("PivotArm/P", PIVOT_ARM_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("PivotArm/I", PIVOT_ARM_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("PivotArm/D", PIVOT_ARM_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("PivotArm/FF", PIVOT_ARM_PID[3]);


    private State state = State.MANUAL;
    private double setpoint = 0;

    private final PivotArmIO io;

    // Create a Mechanism2d visualization of the arm
    private MechanismLigament2d armMechanism;

    public PivotArm(PivotArmIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("PivotArm", inputs);

        armMechanism.setAngle(inputs.angle);

        // Update the PID constants if they have changed
        if (p.get() != io.getP()) 
            io.setP(p.get());
        
        if (i.get() != io.getI())
            io.setI(i.get());
        
        if (d.get() != io.getD())
            io.setD(d.get());
        
        if (ff.get() != io.getFF())
            io.setFF(ff.get());
        
        // Log Inputs
        Logger.getInstance().processInputs("PivotArm", inputs);
    }

    public void setVoltage(double motorVolts) {
        // limit the arm if its past the limit
        if (io.getAngle() > io.PIVOT_ARM_MAX_ANGLE && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getAngle() < io.PIVOT_ARM_MIN_ANGLE && motorVolts < 0) {
            motorVolts = 0;
        }
        
        io.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed * 12);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getAngle() - setpoint) < PIVOT_ARM_PID_TOLERANCE;
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

    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return armMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", 2, 0, 5, new Color8Bit(Color.kAqua));
    }
    
}

