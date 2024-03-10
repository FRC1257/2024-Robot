package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static final double RIGHT_MOTOR_MIN_SPEED = 0;
  public static final double RIGHT_MOTOR_MAX_SPEED = 0;
  public static final double LEFT_MOTOR_MIN_SPEED = 0;
  public static final double LEFT_MOTOR_MAX_SPEED = 0;
  
  public static double defaultShooterSpeedRPM = 1000;

  public static class ShooterSimConstants {
    public static final double RIGHT_MOTOR_MIN_SPEED = 0;
    public static final double RIGHT_MOTOR_MAX_SPEED = 0;
    public static final double LEFT_MOTOR_MIN_SPEED = 0;
    public static final double LEFT_MOTOR_MAX_SPEED = 0;

    public static final double rightEncoder = 0.0;
    public static final double leftEncoder = 0.0;

    public static class ShooterPhysicalConstants {
      public static final double RIGHT_MOTOR_MIN_SPEED = 0;
      public static final double RIGHT_MOTOR_MAX_SPEED = 0;
      public static final double LEFT_MOTOR_MIN_SPEED = 0;
      public static final double LEFT_MOTOR_MAX_SPEED = 0;

    }

    // encoder / flywheelReduction = flywheel
    public static double flywheelReduction = 1.0;
    public static double shooterToleranceRPM = 100.0;

    public static double wheelRadiusM = 0.05;
    public static double wheelMassKg = 0.2;
    public static double momentOfInertia = 0.5 * wheelMassKg * wheelRadiusM * wheelRadiusM;

    public record FlywheelConstants(
        boolean inverted,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA) {
    }

    public record FeederConstants(int id, boolean inverted) {
    }
  }
}
