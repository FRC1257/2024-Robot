package frc.robot.subsystems.pivotArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;

public class PivotArmConstants {

  public static final double POSITION_CONVERSION_FACTOR = 21.0 / 35.0;
  public static final double PIVOT_ARM_ROTATION_DIAM_M = 1;

  public static final double[] PIVOT_ARM_PID_REAL = { 3.6, 0, 0, 0.01 };
  public static final double[] PIVOT_ARM_FEEDFORWARD_REAL = { 0, 0.45, 0, 0 };

  public static final double PIVOT_ARM_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double PIVOT_ARM_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double PIVOT_ARM_OFFSET = 1.1;// 1.14;

  public static final double PIVOT_MAX_PID_TIME = 3;

  public static final double PIVOT_ARM_MAX_ANGLE = Units.degreesToRadians(110.0);
  public static final double PIVOT_ARM_MIN_ANGLE = Units.degreesToRadians(2.0);

  public static final double PIVOT_AMP_ANGLE = Units.degreesToRadians(105.0);
  public static final double PIVOT_SUBWOOFER_ANGLE = Units.degreesToRadians(38);
  public static final double PIVOT_SUBWOOFER_SIDE_ANGLE = Units.degreesToRadians(42);
  public static final double PIVOT_PODIUM_ANGLE = Units.degreesToRadians(53.0);
  public static final double PIVOT_TRAP_ANGLE = Units.degreesToRadians(30.0);
  public static final double PIVOT_ARM_INTAKE_ANGLE = Units.degreesToRadians(2.0);

  public static final double RAMP_RATE = 0.5;
  public static final double STEP_VOLTAGE = 3.0;

  public static class PivotArmSimConstants {
    public static final double[] kPivotSimPID = { 15, 0, 0, 0 };

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