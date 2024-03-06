package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.util.Units;

public class PivotArmConstants {

    public static final double POSITION_CONVERSION_FACTOR = 1;
    public static final double PIVOT_ARM_ROTATION_DIAM_M = 1;

    public static final double[] PIVOT_ARM_PID_REAL = {0.25, 0, 0, 0};
    public static final double PIVOT_ARM_PID_TOLERANCE = Units.degreesToRadians(1);

    public static final double PIVOT_ARM_MAX_ANGLE = Units.degreesToRadians(120.0);
    public static final double PIVOT_ARM_MIN_ANGLE = Units.degreesToRadians(0.0);
    public static final double PIVOT_ARM_AMP_ANGLE = Units.degreesToRadians(90);

    public static class PivotArmSimConstants {
      public static final double[] kPivotSimPID = {15, 0, 0, 0};

      public static final int kMotorPort = 2;
      public static final int kEncoderAChannel = 2;
      public static final int kEncoderBChannel = 3;

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      // = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 1 / 4096;

      public static final double kArmReduction = 200;
      public static final double kArmMass = 10.0; // Kilograms
      public static final double kArmLength = Units.inchesToMeters(20);
      public static final double kMinAngleRads = Units.degreesToRadians(0);
      public static final double kMaxAngleRads = Units.degreesToRadians(180);
    }
  }