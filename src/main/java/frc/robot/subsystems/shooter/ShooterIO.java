package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double leftShooterPositionRotations = 0.0;
    public double leftFlywheelVelocityRPM = 0.0;
    public double leftFlywheelAppliedVolts = 0.0;
    public double leftFlywheelOutputCurrent = 0.0;

    public double rightFlywheelPositionRotations = 0.0;
    public double rightFlywheelVelocityRPM = 0.0;
    public double rightFlywheelAppliedVolts = 0.0;
    public double rightFlywheelOutputCurrent = 0.0;
  }

  /** Update inputs */
  default void updateInputs(ShooterIOInputs inputs) {}

  default void setRPM(double rpm) {}

  default void setVoltage(double voltage) {}

  //make a setRPM that doesn't rely on PID

  default void setBrakeMode(boolean enabled) {}

  default void setCharacterizationVoltage(double volts) {}

  default void setPID(double p, double i, double d) {}

  default void setFF(double s, double v, double a) {}

  default void stop() {}

  default void run(double speed) {
    setVoltage(speed * 12);
  }
}
