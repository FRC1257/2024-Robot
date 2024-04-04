package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds

  // public static final double periodicTime = 0.02;
  // public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

  //public static final double kDirectionSlewRate = 1.2; // radians per second
  //public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
  //public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

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
  // Subject to change depending on what robot side is the front
  public static final double kFrontLeftChassisAngularOffset = Math.PI;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = 0;

  // SPARK MAX CAN IDs
  // Subject to change depending on what robot side is the front
  public static final int kFrontLeftDrivingCanId = 3;
  public static final int kRearLeftDrivingCanId = 7;
  public static final int kFrontRightDrivingCanId = 1;
  public static final int kRearRightDrivingCanId = 5;

  //drive motors are timing out for some reason in the logs

  public static final int kFrontLeftTurningCanId = 4;
  public static final int kRearLeftTurningCanId = 8;
  public static final int kFrontRightTurningCanId = 2;
  public static final int kRearRightTurningCanId = 6;

  //public static final boolean kGyroReversed = false;
  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 1.5;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 2;

  public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared,
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  public static final double kSlowModeConstant = 0.5;
  public static final double kTurnSpeakerP = 1.3;
  public static final double kTurnSpeakerI = 0;
  public static final double kTurnSpeakerD = 0;
  public static final double kTurnSpeakerTolerance = 0.05;
  public static final double kTurnSpeakerRateTolerance = 0.02;

  public static final double angleThreshold = 3;
  
}