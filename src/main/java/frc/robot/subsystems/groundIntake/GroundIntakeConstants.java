package frc.robot.subsystems.groundIntake;


public class GroundIntakeConstants {
    public static class GroundIntakeSimConstants {
      public static final int kMotorPort = 0;

      public static final double kGroundIntakeP = 0.001;
      public static final double kGroundIntakeI = 0.0;
      public static final double kGroundIntakeD = 0.0;
      
      public static final double kGroundIntakeGearing = 0.0;
      public static final double kGroundIntakeDrumRadius = 0.03; 
      public static final double kCarriageMass = 0.15; // Mass in Kg
      public static final double kMomentOfInertia = 0.5 * kCarriageMass * kGroundIntakeDrumRadius * kGroundIntakeDrumRadius; // Moment of inertia represents how resistant to force something is

    }

    public static class GroundIntakePhysicalConstants {
      public static final double kGroundIntakeP = 0.001;
      public static final double kGroundIntakeI = 0.0;
      public static final double kGroundIntakeD = 0.0;
    }

    public static final double GROUND_INTAKE_IN_VOLTAGE = 9.2;
    public static final double GROUND_INTAKE_WEAK_IN_VOLTAGE = 4.5;
    public static final double GROUND_INTAKE_OUT_VOLTAGE = -9;
  }