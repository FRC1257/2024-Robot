package frc.robot.subsystems.indexer;

import frc.robot.Constants;

public final class IndexerConstants {
  public static class IndexerSimConstants {
    public static final int kMotorPort = 0;
    public static final int kBreakBeamSensorChannel = 0;
    public static final int kJoystickPort = 0;

    public static final double kIndexerP = 0.001;
    public static final double kIndexerI = 0.0;
    public static final double kIndexerD = 0.0;

    public static final double kIndexerS = 0.0;
    public static final double kIndexerG = 0.0;
    public static final double kIndexerV = 0.0;
    public static final double kIndexerA = 0.0;
    // Not sure what these three are or if they're needed
    public static final double kIndexerGearing = 0.0;
    public static final double kIndexerDrumRadius = 0.03; 
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia = 0.5 * kCarriageMass * kIndexerDrumRadius * kIndexerDrumRadius; // Moment of inertia represents how resistant to force something is

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public static final double kIndexerEncoderDistPerPulse = 2.0 * Math.PI * kIndexerDrumRadius / 4096;
  }

  public static class IndexerPhysicalConstants {
    // Not sure if these two necessary
    public static final double GROUND_STOP_BUFFER = 0.0;
    public static final double GROUND_TOLERANCE = 0.0;

    public static final double kIndexerP = 0.001;
    public static final double kIndexerI = 0.0;
    public static final double kIndexerD = 0.0;
  }

  public static final double INDEXER_IN_VOLTAGE = 8;
  public static final double INDEXER_IN_VOLTAGE_WEAK = 4.2;
  public static final double INDEXER_OUT_VOLTAGE = -2;
  public static final double INDEXER_TOLERANCE = 1;

  public static final double SHOOTER_UNJAM_TIME = 0.2;

  public static double getIntakeLoopMaxTime() {
    switch (Constants.getRobotMode()) {
      case REAL:
        return 5.0;
      case SIM:
      default:
        return 1.0;
    }
  }
}