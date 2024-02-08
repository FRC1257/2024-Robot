package frc.robot.subsystems.trapPivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.robot.Constants.TrapPivot.*;
import static frc.robot.Constants.TrapPivot.TrapPivotSim.*;

public class TrapPivotIOSim implements TrapPivotIO {
    // The arm gearbox represents a gearbox containing one Vex 775pro motor.
    private final DCMotor gearbox;

    // PID controller helps you move to a setpoint as quickly and smoothly as possible
    private final ProfiledPIDController pidController;
    private final SimpleMotorFeedforward feedForward;

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private SingleJointedArmSim sim;

    // Keeps track of volts applied to arm
    private double appliedVolts = 0;

    // Configure simulation and PID controller
    public TrapPivotIOSim() {
        gearbox = DCMotor.getNeo550(1);
        sim = new SingleJointedArmSim(
            gearbox,
            TRAP_PIVOT_REDUCTION_SIM,
            SingleJointedArmSim.estimateMOI(TRAP_PIVOT_LENGTH_M, TRAP_PIVOT_MASS_KG),
            TRAP_PIVOT_LENGTH_M,
            TRAP_PIVOT_MIN_ANGLE_RADS,
            TRAP_PIVOT_MAX_ANGLE_RADS,
            false,
            0
        );
        pidController = new ProfiledPIDController(TRAP_PIVOT_PID_SIM[0], TRAP_PIVOT_PID_SIM[1], TRAP_PIVOT_PID_SIM[2],
            new TrapezoidProfile.Constraints(TRAP_PIVOT_MAX_VELOCITY, TRAP_PIVOT_MAX_ACCELERATION));
        feedForward = new SimpleMotorFeedforward(TRAP_PIVOT_LENGTH_M, TRAP_PIVOT_MASS_KG);
    }

    // This function runs periodically and updates the inputs
    @Override
    public void updateInputs(TrapPivotIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = sim.getAngleRads();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.setpointAngleRads = pidController.getSetpoint().position;
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
        appliedVolts = volts;
    }

    @Override
    public double getAngleRads() {
        return sim.getAngleRads();
    }

    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(setpoint);

        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(sim.getAngleRads());
        double feedForwardOutput = feedForward.calculate(pidController.getSetpoint().velocity);
        sim.setInputVoltage(pidOutput + feedForwardOutput);
    }

    @Override
    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    @Override
    public double getP() {
        return pidController.getP();
    }

    @Override
    public double getI() {
        return pidController.getI();
    }

    @Override
    public double getD() {
        return pidController.getD();
    }

    @Override
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setP(i);
    }

    @Override
    public void setD(double d) {
        pidController.setP(d);
    }
}
