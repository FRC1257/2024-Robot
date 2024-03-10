package frc.robot.subsystems.intake;

public final class IntakeConstants {

  public static class IntakePhysicalConstants {
    // Not sure if these two necessary
    public static final double GROUND_STOP_BUFFER = 0.0;
    public static final double GROUND_TOLERANCE = 0.0;

    public static final double kIntakeP = 0.001;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
  }

  public static final double INTAKE_IN_VOLTAGE = 8;
  public static final double INTAKE_OUT_VOLTAGE = -8;

  public static final double SHOOTER_UNJAM_TIME = 0.5;
}
