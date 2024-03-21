package frc.robot.subsystems.groundIntake;


public class GroundIntakeConstants {
    public static class GroundIntakeSimConstants {
      public static final int kMotorPort = 0;

      public static final double kGroundIntakeP = 0.001;
      public static final double kGroundIntakeI = 0.0;
      public static final double kGroundIntakeD = 0.0;

      public static final double kGroundIntakeS = 0.0;
      public static final double kGroundIntakeG = 0.0;
      public static final double kGroundIntakeV = 0.0;
      public static final double kGroundIntakeA = 0.0;
      // Not sure what these three are or if they're needed
      public static final double kGroundIntakeGearing = 0.0;
      public static final double kGroundIntakeDrumRadius = 0.03; 
      public static final double kCarriageMass = 0.15; // Mass in Kg
      public static final double kMomentOfInertia = 0.5 * kCarriageMass * kGroundIntakeDrumRadius * kGroundIntakeDrumRadius; // Moment of inertia represents how resistant to force something is

      // distance per pulse = (distance per revolution) / (pulses per revolution)
      // = (Pi * D) / ppr
      public static final double kGroundIntakeEncoderDistPerPulse = 2.0 * Math.PI * kGroundIntakeDrumRadius / 4096;
    }

    public static class GroundIntakePhysicalConstants {
      // Not sure if these two necessary
      public static final double GROUND_STOP_BUFFER = 0.0;
      public static final double GROUND_TOLERANCE = 0.0;
      
      public static final double kGroundIntakeP = 0.001;
      public static final double kGroundIntakeI = 0.0;
      public static final double kGroundIntakeD = 0.0;

      public static final double kGroundIntakeS = 0.0;
      public static final double kGroundIntakeG = 0.0;
      public static final double kGroundIntakeV = 0.0;
      public static final double kGroundIntakeA = 0.0;
    }

    public static final double GROUND_INTAKE_IN_VOLTAGE = 9;
    public static final double GROUND_INTAKE_OUT_VOLTAGE = -9;
  }