package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static final double SHOOTER_SPINUP_TIME = 0.5;
  public static final double SHOOTER_UNJAM_VOLTAGE = -0.5;

  public static final double SHOOTER_FULL_VOLTAGE = 11;
  public static final double SHOOTER_PREP_VOLTAGE = 11;

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

    public static FlywheelConstants leftShooter = switch (Constants.currentMode) {
      case REAL -> new FlywheelConstants(false, 0.0008, 0.4, 0.00001, 0.33329, 0.00083, 0.0);
      default -> new FlywheelConstants(false, 0.0008, 0.4, 0.00001, 0.33329, 0.00083, 0.0);
    };

    public static FlywheelConstants rightShooter = switch (Constants.currentMode) {
      case REAL -> new FlywheelConstants(false, 0.0008, 0.4, 0.00001, 0.33329, 0.00083, 0.0);
      default -> new FlywheelConstants(false, 0.0008, 0.4, 0.00001, 0.33329, 0.00083, 0.0);
    };

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
