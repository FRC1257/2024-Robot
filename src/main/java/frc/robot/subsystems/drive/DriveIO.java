package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;
    public double[] gyroRollPitchYawRad = new double[3];
    public double timestamp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  public default double getLeftPositionMeters() {
    return 0.0;
  }

  public default double getRightPositionMeters() {
    return 0.0;
  }

  public default double getRobotAngle() {
    return 0.0;
  }

  public default void zero() {}

  public default void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {}

  public default double getTrackWidth() {
    return 0.0;
  };

}
