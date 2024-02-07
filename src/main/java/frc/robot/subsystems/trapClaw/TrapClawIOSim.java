package frc.robot.subsystems.trapClaw;

import static frc.robot.Constants.TrapClaw.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TrapClawIOSim implements TrapClawIO {
    // Simulates the gearbox of the motor
    private DCMotor gearbox;

    // Keeps track of volts applied to claw
    private double appliedVolts = 0;

    // Simulates the flywheel physics
    private FlywheelSim sim;

    // Instantiate gearbox and flywheel simulation
    public TrapClawIOSim() {
        gearbox = DCMotor.getNeo550(1);
        sim = new FlywheelSim(gearbox, TRAP_CLAW_REDUCTION_SIM, TRAP_CLAW_MOI);
    }

    // Updates inputs periodically based on state of simulation
    @Override
    public void updateInputs(TrapClawIOInputs inputs) {
        sim.update(0.02);
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[]{ sim.getCurrentDrawAmps() };
    }

    // Sets voltage of motor
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
        appliedVolts = volts;
    }
}
