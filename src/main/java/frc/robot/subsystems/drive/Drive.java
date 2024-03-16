// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.useVision;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.kPathConstraints;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthX;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthY;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.util.autonomous.LocalADStarAK;
import frc.robot.util.drive.AllianceFlipUtil;

public class Drive extends SubsystemBase {
  // private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  private static final double MAX_ANGULAR_SPEED = kMaxSpeedMetersPerSecond / DRIVE_BASE_RADIUS;

  public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
      kMaxSpeedMetersPerSecond, DRIVE_BASE_RADIUS, new ReplanningConfig());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  //private final SysIdRoutine sysId;

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      rawGyroRotation,
      lastModulePositions);

  private SysIdRoutine sysId;
  private SysIdRoutine turnRoutine;

  private Rotation2d simRotation = new Rotation2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      VisionIO visionIO
    ) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    SparkMaxOdometryThread.getInstance().start();
    
    this.visionIO = visionIO;

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        config,
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    Pathfinding.ensureInitialized();

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> 
            {
              for (Module module : modules) {
                module.runCharacterization(volts.in(Volts), 0);
              }
            
            }, null, this));

    turnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
            volts -> 
            {
              for (Module module : modules) {
                module.runCharacterization(0, volts.in(Volts));
              }
            
            }, null, this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);

    if (useVision) {
      visionIO.updateInputs(visionInputs, getPose());
      Logger.processInputs("Vision", visionInputs);
      poseEstimator.addVisionMeasurement(visionInputs.estimate, visionInputs.timestamp, visionIO.getEstimationStdDevs(getPose()));
    }

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
    }

    Logger.recordOutput("FieldVelocity", getFieldVelocity());
    
    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      rawGyroRotation = simRotation;
    }
    
    poseEstimator.update(rawGyroRotation, modulePositions);
    odometry.update(rawGyroRotation, modulePositions);

    Logger.recordOutput("Odometry/Odometry", odometry.getPoseMeters());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    simRotation = simRotation.rotateBy(
        Rotation2d.fromRadians(
            discreteSpeeds.omegaRadiansPerSecond * 0.02));
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxSpeedMetersPerSecond);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void resetYaw() {
    gyroIO.zeroAll();
    setPose(AllianceFlipUtil.apply(new Pose2d()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }


  /** Returns the module states (turn angles and driveZ velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getRotation());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  /* public Rotation2d getRotation() {
    return getPose().getRotation();
  } */
  public Rotation2d getRotation() {
    return new Rotation2d(gyroIO.getYawAngle());
    //return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command turnQuasistatic(SysIdRoutine.Direction direction) {
    return turnRoutine.quasistatic(direction);
  }

  public Command turnDynamic(SysIdRoutine.Direction direction) {
    return turnRoutine.dynamic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
  
  
  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxSpeedMetersPerSecond;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
    };
  }

  /* Configure trajectory following */
  public Command goToPose(Pose2d target_pose, double end_velocity, double time_before_turn) {
    return AutoBuilder.pathfindToPose(target_pose, kPathConstraints, end_velocity, time_before_turn);
  }

  public Command goToPose(Pose2d target_pose) {
    return AutoBuilder.pathfindToPose(target_pose, kPathConstraints, 0.0, 1);
  }

  public Command getAuto(String nameString) {
    return AutoBuilder.buildAuto(nameString);
  }

  public Command runTrajectory(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public Command pathfindToTrajectory(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, kPathConstraints);
  }

  public Command goToThaPose(Pose2d endPose) {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      getPose(),
      endPose
    );

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      kPathConstraints, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    return AutoBuilder.followPath(path);
  }

  public Command goToNote(){ //not supported for Sim yet
    return DriveCommands.turnToNote(this).andThen(goToPose(visionIO.calculateNotePose(getPose(), visionIO.calculateNoteTranslation(visionInputs))));
    //might have to negate direction or angle due to orientation of the robot's intake
  }

  public Translation2d calculateNoteTranslation() {
    return visionIO.calculateNoteTranslation(visionInputs);
  }

  public Pose2d calculateNotePose(Pose2d robotPose, Translation2d noteTranslation) {
    return visionIO.calculateNotePose(robotPose, noteTranslation);
  }

  public Rotation2d getAngleToNote() {
    return visionIO.getAngleToNote();
  }
  
}