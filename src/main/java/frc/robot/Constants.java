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

    // Trap Arm
    public final static int TRAP_PIVOT_ID = 5;
    public final static int TRAP_CLAW_ID = 6;
  };

  // CHANGE CONSTANTS RIGHT NOW YOU STUPID IDIOT
  public static class TrapPivot {
    // Arm measurements
    public final static double TRAP_PIVOT_LENGTH_M = 1;
    public final static double TRAP_PIVOT_MASS_KG = 1;
    public final static double TRAP_PIVOT_MIN_ANGLE_RADS = 0;
    public final static double TRAP_PIVOT_MAX_ANGLE_RADS = 2;
    public final static double TRAP_PIVOT_ANGLE_THRESHOLD = (TRAP_PIVOT_MIN_ANGLE_RADS + TRAP_PIVOT_MAX_ANGLE_RADS) / 2;
    public final static double TRAP_PIVOT_SIMULATION_MAX_ANGLE_RADS = Math.PI;
    public final static double TRAP_PIVOT_SIMULATION_MIN_ANGLE_RADS = 0;
    // PID tolerance
    public final static double TRAP_PIVOT_PID_TOLERANCE = 0.1;

    public static class TrapPivotSim {
      // Trap pivot measurements
      public final static double TRAP_PIVOT_REDUCTION_SIM = 1;

      // PID constants
      public final static double[] TRAP_PIVOT_PID_SIM = { 15, 0, 0 };
      public final static double TRAP_PIVOT_MAX_VELOCITY = 5.45;
      public final static double TRAP_PIVOT_MAX_ACCELERATION = 2.45;
    }

    public static class TrapPivotReal {
      // PID constants
      public final static double[] TRAP_PIVOT_PID_REAL = { 1.257, 0, 0 };
      public final static double TRAP_PIVOT_REDUCTION_REAL = 1;
    }
  }

  // CHANGE THESE CONSTANTS TOO YOU FUCKING IDIOT
  public static class TrapClaw {
    public final static double TRAP_CLAW_REDUCTION_REAL = 1;
    public final static double TRAP_CLAW_REDUCTION_SIM = 1;
    public final static double TRAP_CLAW_SPEED = 0.5;

    // Moment of inertia calculations
    public final static double TRAP_CLAW_WHEEL_MASS_KG = 0.1;
    public final static double TRAP_CLAW_WHEEL_RADIUS_M = 0.05;
    public final static double TRAP_CLAW_MOI = 0.5 * TRAP_CLAW_WHEEL_MASS_KG * TRAP_CLAW_WHEEL_RADIUS_M * TRAP_CLAW_WHEEL_RADIUS_M;
  }

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public final static int NEO_550_CURRENT_LIMIT = 25; // amps
  public final static int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  public final static int NEO_CURRENT_LIMIT = 80; // amps
}
