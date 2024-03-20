package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim leftSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), flywheelReduction, momentOfInertia);
  private final FlywheelSim rightSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), flywheelReduction, momentOfInertia);

  private final PIDController leftController =
      new PIDController(
          leftShooter.kP(), leftShooter.kI(), leftShooter.kD());
  private final PIDController rightController =
      new PIDController(
          rightShooter.kP(), rightShooter.kI(), rightShooter.kD());
  private SimpleMotorFeedforward leftFF =
      new SimpleMotorFeedforward(
          leftShooter.kS(), leftShooter.kV(), leftShooter.kA());
  private SimpleMotorFeedforward rightFF =
      new SimpleMotorFeedforward(
          rightShooter.kS(), rightShooter.kV(), rightShooter.kA());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRPM = null;
  private Double rightSetpointRPM = null;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    leftSim.update(0.02);
    rightSim.update(0.02);
    // control to setpoint
    if (leftSetpointRPM != null) {
      leftAppliedVolts =
          leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRPM)
              + leftFF.calculate(leftSetpointRPM);
      leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0)); //we are limiting the voltage
      Logger.recordOutput("Shooter Voltage Left", MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
      //leftSim.setInputVoltage(leftAppliedVolts);
    }
    if (rightSetpointRPM != null) {
      rightAppliedVolts =
          rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRPM)
              + rightFF.calculate(rightSetpointRPM);
      rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
      Logger.recordOutput("Shooter Voltage Right", MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
      //rightSim.setInputVoltage(rightAppliedVolts);
    } //still not setting input voltage high enough

    inputs.leftShooterPositionRotations +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.leftFlywheelVelocityRPM = leftSim.getAngularVelocityRPM();
    inputs.leftFlywheelAppliedVolts = leftAppliedVolts;
    inputs.leftFlywheelOutputCurrent = leftSim.getCurrentDrawAmps();

    inputs.rightFlywheelPositionRotations +=
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.rightFlywheelVelocityRPM = rightSim.getAngularVelocityRPM();
    inputs.rightFlywheelAppliedVolts = rightAppliedVolts;
    inputs.rightFlywheelOutputCurrent = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void setLeftRPM(double rpm) {
    leftSetpointRPM = rpm;
  }

  @Override
  public void setRightRPM(double rpm) {
    rightSetpointRPM = rpm;
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftSetpointRPM = null;
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightSetpointRPM = null;
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
  public void setLeftPID(double p, double i, double d) {
    leftController.setPID(p, i, d);
  }

  @Override
  public void setLeftFF(double s, double v, double a) {
    leftFF = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void setRightPID(double p, double i, double d) {
    rightController.setPID(p, i, d);
  }

  @Override
  public void setRightFF(double s, double v, double a) {
    rightFF = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void stop() {
    setRPM(0.0, 0.0);
  }
}