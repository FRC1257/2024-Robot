// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;


  public static final boolean tuning = true;

  public static final boolean tuningMode = true;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /* Test bot */
    TEST,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidthX = Units.inchesToMeters(26.5);
    public static final double kTrackWidthY = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidthX / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidthX / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidthX / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidthX / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
    
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared,
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    );

    public static final double kSlowModeConstant = 0.5;
    public static final double kTurnSpeakerP = 0.9;
    public static final double kTurnSpeakerI = 0;
    public static final double kTurnSpeakerD = 0;
    public static final double kTurnSpeakerTolerance = 0.05;
    public static final double kTurnSpeakerRateTolerance = 0.02;
  }

  public static class BuildConstants {
    public static int DIRTY = 1;
    public static String MAVEN_NAME = "Snail";
    public static String BUILD_DATE = "12/57";
    public static String GIT_SHA = "Snail";
    public static String GIT_DATE = "Snail";
    public static String GIT_BRANCH = "PivotArm";
  }
  public static class Drivetrain {
    // drivetrain constants
    public static double DRIVE_TRACK_WIDTH_M = 0.86;// 0.66; // m
    public static double DRIVE_WHEEL_DIAM_M = 0.1524; // m
    public static double DRIVE_GEARBOX_REDUCTION = 10.71;

    // driving modifiers
    public static double DRIVE_SLOW_TURN_MULT = 0.25;
    public static double DRIVE_SLOW_FORWARD_MULT = 0.25;

    // closed loop driving
    public static double DRIVE_CLOSED_MAX_VEL = 4.0; // m/s
    public static double DRIVE_CLOSED_MAX_ROT_TELEOP = 360.00; //
    public static double DRIVE_CLOSED_MAX_ROT_AUTO = 100.0; // deg/s
    public static double DRIVE_CLOSED_MAX_ACC = 1.5; // m/s^2

    // trajectory following
    public static double DRIVE_TRAJ_MAX_VEL = 8.0; // m/s
    public static double DRIVE_TRAJ_MAX_ACC = 0.7515; // .75; // m/s^2
    public static double DRIVE_TRAJ_RAMSETE_B = 2.0; // don't change
    public static double DRIVE_TRAJ_RAMSETE_ZETA = 0.7;

    public static double DRIVE_TRAJ_KV = 0.0; // don't change
    public static double DRIVE_TRAJ_KA = 0.0; // don't change
    public static double DRIVE_TRAJ_KS = 0.0; // don't change

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // aligning
    public static double DRIVE_ALIGN_MAX_VEL = 0.75; // m/s
    public static double DRIVE_ALIGN_MAX_ACC = 0.350; // .75; // m/s^2

    // linear position PID
    public static double[] DRIVE_DIST_PID = { 3.50, 0.0, 0.0 };
    public static double DRIVE_DIST_ANGLE_P = 0.1;
    public static double DRIVE_DIST_TOLERANCE = 0.01;
    public static double DRIVE_DIST_MAX_OUTPUT = 0.6;

    // angular position PID works for test bot
    public static double[] DRIVE_ANGLE_PID = { 0.045, 0.1, 0.005 }; // 0.055
    public static double DRIVE_ANGLE_TOLERANCE = 0.5;
    public static double DRIVE_ANGLE_MAX_OUTPUT = 0.6;

    // velocity PID (for closed loop, profiling, and trajectory)
    public static int DRIVE_VEL_SLOT = 0;
    public static double DRIVE_VEL_LEFT_P = 0.25;
    public static double DRIVE_VEL_LEFT_F = 0.25;
    public static double DRIVE_VEL_RIGHT_P = 0.25;
    public static double DRIVE_VEL_RIGHT_F = 0.25;

    // profiling position PID (for further refinement of tracking)
    public static double DRIVE_PROFILE_LEFT_P = 0.1;
    public static double DRIVE_PROFILE_RIGHT_P = 0.1;

    // vision PID
    public static final double TRACKED_TAG_ROTATION_KP = 0.375;
    public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2; // P (Proportional) constant of a PID loop
    public static final double APRILTAG_POWER_CAP = 0.75;
  };


  public static class Shooter {
    public static final double RIGHT_MOTOR_MIN_SPEED = 0;
    public static final double RIGHT_MOTOR_MAX_SPEED = 0;
    public static final double LEFT_MOTOR_MIN_SPEED = 0;
    public static final double LEFT_MOTOR_MAX_SPEED = 0;

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
    }
  }



  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second 

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }



  public static class Vision {
    public static final String kCameraName = "Front_Camera";
    public static final String kBackCameraName = "Back_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));
    public static final Transform3d kBackRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 135));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class Intake {
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
    }
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class ElectricalLayout {
    // Controllers
    public final static int CONTROLLER_DRIVER_ID = 0;
    public final static int CONTROLLER_OPERATOR_ID = 1;

    // Drivetrain Main
    public final static int DRIVE_FRONT_LEFT = 1;
    public final static int DRIVE_FRONT_RIGHT = 2;
    public final static int DRIVE_BACK_LEFT = 3;
    public final static int DRIVE_BACK_RIGHT = 4;

    // PLACEHOLDER Intake
    public final static int INTAKE_MOTOR = 0;

    public final static int INTAKE_BREAK_BEAM = 0;
  };

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public final static int NEO_550_CURRENT_LIMIT = 25; // amps
  public final static int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  public final static int NEO_CURRENT_LIMIT = 80; // amps


  public static class ShooterConstants {
    // encoder / flywheelReduction = flywheel
    public static double flywheelReduction = (1.0 / 2.0);
    public static double shooterToleranceRPM = 50.0;
    public static double defaultShooterSpeedRPM = 1000;
    
    public static double wheelRadiusM = 0.05;
    public static double wheelMassKg = 0.2;
    public static double momentOfInertia = 0.5 * wheelMassKg * wheelRadiusM * wheelRadiusM;
  
    public static FlywheelConstants leftShooter =
        switch (Constants.currentMode) {
          default -> new FlywheelConstants(2, false, 0.001, 0.01, 0.0, 0.33329, 0.00083, 0.0);
        };
  
    public static FlywheelConstants rightShooter =
        switch (Constants.currentMode) {
          default -> new FlywheelConstants(1, false, 0.001, 0.01, 0.0, 0.33329, 0.00083, 0.0);
        };
        
    public record FlywheelConstants(
        int id,
        boolean inverted,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA) {}
  
    public record FeederConstants(int id, boolean inverted) {}
  }
  

  
  public static class PivotArm {
    // CHANGE CONSTANTS, THESE ARE TEMPORARY
    public static final int PIVOT_ARM_ID = 0;
    public static final int LEFT_SLAVE_ID = 1;
    public static final int RIGHT_SLAVE_FRONT_ID = 2;
    public static final int RIGHT_SLAVE_BACK_ID = 3;

    public static final double POSITION_CONVERSION_FACTOR = 1;
    public static final double PIVOT_ARM_ROTATION_DIAM_M = 1;

    public static final double[] PIVOT_ARM_PID_REAL = {0.25, 0, 0, 0};
    public static final double PIVOT_ARM_PID_TOLERANCE = Units.degreesToRadians(1);

    public static final double PIVOT_ARM_MAX_ANGLE = Units.degreesToRadians(105.0);
    public static final double PIVOT_ARM_MIN_ANGLE = Units.degreesToRadians(0.0);

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
      public static final double kMinAngleRads = Units.degreesToRadians(-175);
      public static final double kMaxAngleRads = Units.degreesToRadians(255);
    }
  }


}
