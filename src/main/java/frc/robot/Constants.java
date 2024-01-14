// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

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

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
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

  public static class Vision {
    public static final String kCameraName = "Front_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
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
  };

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public final static int NEO_550_CURRENT_LIMIT = 25; // amps
  public final static int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  public final static int NEO_CURRENT_LIMIT = 80; // amps
}
