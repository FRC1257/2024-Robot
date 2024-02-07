package frc.robot.subsystems.trapClaw;

import static frc.robot.Constants.TrapClaw.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TrapClawIOSim implements TrapClawIO {
    private DCMotor gearbox;
    private FlywheelSim sim;

    public TrapClawIOSim() {
        gearbox = DCMotor.getNeo550(1);
        sim = new FlywheelSim(gearbox, TRAP_CLAW_REDUCTION_SIM, 0.1);
    }

    @Override
    public void updateInputs(TrapClawIOInputs inputs) {
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = new double[]{ sim.getCurrentDrawAmps() };
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }
}
