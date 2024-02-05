package frc.robot.subsystems.trapPivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.robot.Constants.TrapPivot.TrapPivotSim.*;

public class TrapPivotIOSim implements TrapPivotIO {
    // The arm gearbox represents a gearbox containing one Vex 775pro motor.
    private final DCMotor gearbox = DCMotor.getVex775Pro(1);

    // Standard classes for controlling our arm
    private final ProfiledPIDController pidController;

    // PID setpoint
    private double setpoint = 0;

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private SingleJointedArmSim sim;

    public TrapPivotIOSim() {
        sim = new SingleJointedArmSim(
            gearbox,
            TRAP_ARM_REDUCTION,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG),
            ARM_LENGTH_M,
            MIN_ANGLE_RADS,
            MAX_ANGLE_RADS,
            false,
            0
        );
        pidController = new ProfiledPIDController(TRAP_PIVOT_PID_SIM[0], TRAP_PIVOT_PID_SIM[1], TRAP_PIVOT_PID_SIM[2],
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    }

    @Override
    public void updateInputs(TrapPivotIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = sim.getAngleRads();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public double getAngleRads() {
        return sim.getAngleRads();
    }

    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(sim.getAngleRads());
        double feedForwardOutput = 0;

        sim.setInputVoltage(feedForwardOutput + pidOutput);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(sim.getAngleRads() - setpoint) <= PID_TOLERANCE;
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
