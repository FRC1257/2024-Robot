package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.flywheelReduction;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.momentOfInertia;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim flywheelSim =
      new FlywheelSim(DCMotor.getNeoVortex(2), flywheelReduction, momentOfInertia);

  private final PIDController controller =
      new PIDController(
          shooter.kP(), shooter.kI(), shooter.kD());
  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          shooter.kS(), shooter.kV(), shooter.kA());

  private double volts = 0.0;

  private Double setpointRPM = null;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(0.02);

    // control to setpoint
    if (setpointRPM != null) {
      volts =
          controller.calculate(flywheelSim.getAngularVelocityRPM(), setpointRPM)
              + feedforward.calculate(setpointRPM);
      flywheelSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    inputs.leftShooterPositionRotations +=
        Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.leftFlywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.leftFlywheelAppliedVolts = volts;
    inputs.leftFlywheelOutputCurrent = flywheelSim.getCurrentDrawAmps();

    inputs.rightFlywheelPositionRotations +=
        Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.rightFlywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.rightFlywheelAppliedVolts = volts;
    inputs.rightFlywheelOutputCurrent = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setRPM(double rpm) {
    setpointRPM = rpm;
  }

  @Override
  public void setCharacterizationVoltage(double volts) {
    setpointRPM = null;
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void setFF(double s, double v, double a) {
    feedforward = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}