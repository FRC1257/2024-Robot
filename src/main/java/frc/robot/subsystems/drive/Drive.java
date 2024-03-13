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

import static frc.robot.Constants.useVision;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.kPathConstraints;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthX;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthY;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.util.autonomous.LocalADStarAK;
import frc.robot.util.drive.SwerveUtils;

public class Drive extends SubsystemBase {
  // private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  private static final double MAX_ANGULAR_SPEED = kMaxSpeedMetersPerSecond / DRIVE_BASE_RADIUS;

  public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
      kMaxSpeedMetersPerSecond, DRIVE_BASE_RADIUS, new ReplanningConfig());

  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  
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
  private Pose2d lastPose = new Pose2d();
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry;

  private Rotation2d simRotation = new Rotation2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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

    odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(gyroIO.getYawAngle()),
      getModulePositions()
    );
    
    this.visionIO = visionIO;

    // Configure AutoBuilder for PathPlanner
    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (ChassisSpeeds speeds) -> runVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

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

    
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
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
      // rawGyroRotation = simRotation;
    }
    
    poseEstimator.update(rawGyroRotation, modulePositions);
    odometry.update(rawGyroRotation, modulePositions);

    Logger.recordOutput("Odometry/Odometry", odometry.getPoseMeters());
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void resetYaw() {
    gyroIO.zeroAll();
    setPose(lastPose);
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

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyroIO.getYawAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    setDesiredState(swerveModuleStates);
  }

  public void setDesiredState(SwerveModuleState[] states) {
    /* SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond); */
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  public void  runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    
    SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

    setDesiredState(setpointStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (Module module : modules) {
      module.resetEncoders();
    }
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
        DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), getRotation());
  }

  /** 
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
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

  /* public Command goToNote(){ //not supported for Sim yet
    return DriveCommands.turnToNote(this).andThen(goToPose(visionIO.calculateNotePose(getPose(), visionIO.calculateNoteTranslation(visionInputs))));
    //might have to negate direction or angle due to orientation of the robot's intake
  } */

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