package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static class IntakeSimConstants {
    public static final int kMotorPort = 0;
    public static final int kBreakBeamSensorChannel = 0;
    public static final int kJoystickPort = 0;

    public static final double kIntakeP = 0.001;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;

    public static final double kIntakeS = 0.0;
    public static final double kIntakeG = 0.0;
    public static final double kIntakeV = 0.0;
    public static final double kIntakeA = 0.0;
    // Not sure what these three are or if they're needed
    public static final double kIntakeGearing = 0.0;
    public static final double kIntakeDrumRadius = 0.03; 
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia = 0.5 * kCarriageMass * kIntakeDrumRadius * kIntakeDrumRadius; // Moment of inertia represents how resistant to force something is

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public static final double kIntakeEncoderDistPerPulse = 2.0 * Math.PI * kIntakeDrumRadius / 4096;
  }

  public static class IntakePhysicalConstants {
    // Not sure if these two necessary
    public static final double GROUND_STOP_BUFFER = 0.0;
    public static final double GROUND_TOLERANCE = 0.0;

    public static final double kIntakeP = 0.001;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
  }
}
