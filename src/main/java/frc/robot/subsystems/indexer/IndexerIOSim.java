package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.IndexerSimConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ElectricalLayout;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.2, kMomentOfInertia);
    private PIDController controller = new PIDController(0, 0, 0);

    private double appliedVoltage = 0.0;
    private double desiredSpeed;

    public IndexerIOSim() {
        setPIDConstants(kIndexerP, kIndexerI, kIndexerD);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        sim.update(0.02);
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
        inputs.tempCelcius = new double[] { 60 };
        inputs.speedSetpoint = desiredSpeed;
        inputs.breakBeam = true;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        sim.setInputVoltage(0.0);
    }

    @Override
    public void setSpeed(double speed) {
        desiredSpeed = speed;
        setVoltage(controller.calculate(sim.getAngularVelocityRadPerSec(), speed));
    }

    @Override
    public double getSpeed() {
        return sim.getAngularVelocityRadPerSec();
    }

    @Override
    public void setP(double p) {
        controller.setP(p);
    }

    @Override
    public void setI(double i) {
        controller.setI(i);
    }

    @Override
    public void setD(double d) {
        controller.setD(d);
    }

    @Override
    public double getP() {
        return controller.getP();
    }

    @Override
    public double getI() {
        return controller.getI();
    }

    @Override
    public double getD() {
        return controller.getD();
    }

    @Override
    public void setPIDConstants(double p, double i, double d) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
    }

    @Override
    public boolean isIntaked() {
        return false;
    }

}