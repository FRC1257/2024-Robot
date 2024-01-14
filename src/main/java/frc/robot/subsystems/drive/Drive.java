package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drivetrain.*;

import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.util.ArcadeDrive;

public class Drive extends SubsystemBase {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

  private final Field2d m_field = new Field2d();

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionIOInputs = new VisionIOInputsAutoLogged();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  private RamseteController ramseteController = new RamseteController(DRIVE_TRAJ_RAMSETE_B, DRIVE_TRAJ_RAMSETE_ZETA);
  private Trajectory trajectory;
  private Timer pathTimer = new Timer();

  private PIDController turnController = new PIDController(DRIVE_ANGLE_PID[0], DRIVE_ANGLE_PID[1], DRIVE_ANGLE_PID[2]);
  private double angleTarget = 0.0;

  private boolean slowMode = false;

  private DifferentialDriveKinematics driveKinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;

  public enum States {
    MANUAL,
    TRAJECTORY,
    TURN_ANGLE
  }

  private States defaultState = States.MANUAL;
  private States state = States.MANUAL;

  /** Creates a new Drive. */
  public Drive(DriveIO io, VisionIO visionIO, Pose2d initialPoseMeters) {
    this.io = io;
    this.visionIO = visionIO;

    driveKinematics = new DifferentialDriveKinematics(io.getTrackWidth());
    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData("Field", m_field);
    poseEstimator = new DifferentialDrivePoseEstimator(driveKinematics,
        Rotation2d.fromDegrees(-io.getRobotAngle()), io.getLeftPositionMeters(),
        io.getRightPositionMeters(), initialPoseMeters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    visionIO.updateInputs(visionIOInputs, getPose());
    Logger.processInputs("Vision", visionIOInputs);

    // Update odometry and log the new pose
    odometry.update(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());

    var visionEst = visionIO.getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = visionIO.getEstimationStdDevs(estPose);

          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
    poseEstimator.updateWithTime(inputs.timestamp, new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());

    Logger.recordOutput("Odometry", getPose());

    m_field.setRobotPose(getPose());
  }

  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
  }

  public DifferentialDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);

    if (slowMode) {
      double slow_speed = 0.5;
      io.setVoltage(speeds.left * slow_speed * 12.0, speeds.right * slow_speed * 12.0);
    } else {
      io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    }

  }

  public void followTrajectory() {
    if (trajectory == null || state != States.TRAJECTORY) {
      state = defaultState;
      stop();
      return;
    }

    if (pathTimer.get() > trajectory.getTotalTimeSeconds()) {
      pathTimer.stop();
      pathTimer.reset();

      // stop the bot
      stop();
      return;
    }

    Trajectory.State currentState = trajectory.sample(pathTimer.get());

    ChassisSpeeds chassisSpeeds = ramseteController.calculate(getPose(), currentState);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

    io.setVelocity(wheelSpeeds);
  }

  public void endTrajectory() {
    trajectory = null;
    state = defaultState;
    stop();
  }

  public Command endTrajectoryCommand() {
    return new InstantCommand(() -> endTrajectory(), this);
  }

  public void driveTrajectory(Trajectory trajectory) {
    if (trajectory == null) {
      return;
    }

    zero();
    setRobotPose(trajectory.getInitialPose());

    this.trajectory = trajectory;

    m_field.getObject("traj").setTrajectory(trajectory);

    pathTimer.reset();
    pathTimer.start();

    state = States.TRAJECTORY;
  }

  private void setRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(-io.getRobotAngle()), io.getLeftPositionMeters(),
        io.getRightPositionMeters(), pose);
  }

  /** Stops the drive. */
  public void stop() {
    state = States.MANUAL;
    trajectory = null;
    angleTarget = 0.0;
    io.setVoltage(0.0, 0.0);
  }

  public void toggleSlowMode() {
    slowMode = !slowMode;
  }

  public void startSlowMode() {
    slowMode = true;
  }

  public void stopSlowMode() {
    slowMode = false;
  }

  public void zero() {
    io.zero();
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getVelocityMetersPerSecond() {
    return (getLeftVelocityMeters() + getRightVelocityMeters()) / 2.0;
  }

  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  public boolean isTrajectoryFinished() {
    return state != States.TRAJECTORY;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(new Rotation2d(io.getRobotAngle()), getLeftPositionMeters(),
        getRightPositionMeters(), pose);
  }

  public Command driveTrajectoryCommand(Trajectory trajectory) {
    return new FunctionalCommand(
        () -> driveTrajectory(trajectory),
        () -> followTrajectory(),
        (a) -> stop(),
        this::isTrajectoryFinished,
        this);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMeters(), getRightVelocityMeters());
  }

  public void outputVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void setTurnStart(double angle) {
    setTurnAngle((getPose().getRotation().getDegrees() + angle + 360) % 360);
  }

  public void setTurnAngle(double angle) {
    stop();
    angleTarget = angle;
    state = States.TURN_ANGLE;
    turnController.reset();
    turnController.setSetpoint(chooseClosestTarget());
    turnController.setTolerance(0.5);

  }

  public double chooseClosestTarget() {
    double currentAngle = getPose().getRotation().getDegrees();
    double targetAngle = angleTarget;
    double negativeTargetAngle = angleTarget - 360;

    double currentError = Math.abs(currentAngle - targetAngle);
    double negativeError = Math.abs(currentAngle - negativeTargetAngle);

    if (currentError < negativeError) {
      return targetAngle;
    } else {
      return negativeTargetAngle;
    }
  }

  public void driveTurn() {
    // comment this out while initially tuning

    /*
     * if (pathTimer.get() > 10) {
     * DriverStation.reportError("Turn Command ended", false);
     * state = defaultState;
     * angleSetpoint = defaultSetpoint;
     * frontLeftMotor.set(0);
     * frontRightMotor.set(0);
     * break;
     * }
     */

    double turnOutput = -turnController.calculate(getPose().getRotation().getDegrees(), chooseClosestTarget());

    if (turnController.atSetpoint() && turnController.getVelocityError() < 1
        && turnController.getVelocityError() != 0) {
      state = defaultState;
      angleTarget = 0;
      stop();
      return;
    }

    turnOutput = MathUtil.clamp(turnOutput, -DRIVE_ANGLE_MAX_OUTPUT, DRIVE_ANGLE_MAX_OUTPUT);

    double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(0, turnOutput);
    io.setVoltage(arcadeSpeeds[0] * 12.0, arcadeSpeeds[1] * 12.0);
  }

  public boolean isTurningFinished() {
    return state != States.TURN_ANGLE;
  }

  public Command turnAngleCommand(double angle) {
    return new FunctionalCommand(
        () -> setTurnStart(angle),
        () -> driveTurn(),
        (a) -> stop(),
        this::isTurningFinished,
        this);
  }

  public Command turnExactAngle(double angle) {
    return new FunctionalCommand(
        () -> setTurnAngle(angle),
        () -> driveTurn(),
        (a) -> stop(),
        this::isTurningFinished,
        this);
  }

  /*
   * public RamseteCommand getRamseteCommand(Trajectory trajectory) {
   * return new RamseteCommand(
   * trajectory,
   * this::getPose,
   * ramseteController,
   * new SimpleMotorFeedforward(DRIVE_TRAJ_KS, DRIVE_TRAJ_KV, DRIVE_TRAJ_KA),
   * driveKinematics,
   * this::getWheelSpeeds,
   * new PIDController(0.95, 0, 0),
   * new PIDController(0.95, 0, 0),
   * this::outputVolts,
   * this
   * );
   * }
   */

  /*
   * // Assuming this method is part of a drivetrain subsystem that provides the
   * // necessary methods
   * public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
   * isFirstPath) {
   * return new SequentialCommandGroup(
   * new InstantCommand(() -> {
   * // Reset odometry for the first path you run during auto
   * if (isFirstPath) {
   * this.resetOdometry(traj.getInitialPose());
   * }
   * }),
   * new PPRamseteCommand(
   * traj,
   * this::getPose, // Pose supplier
   * this.ramseteController,
   * new SimpleMotorFeedforward(DRIVE_TRAJ_KS, DRIVE_TRAJ_KV, DRIVE_TRAJ_KA),
   * this.driveKinematics, // DifferentialDriveKinematics
   * this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
   * new PIDController(0, 0, 0), // Left controller. Tune these values for your
   * robot. Leaving them 0 will only
   * // use feedforwards.
   * new PIDController(0, 0, 0), // Right controller (usually the same values as
   * left controller)
   * this::outputVolts, // Voltage biconsumer
   * false, // Should the path be automatically mirrored depending on alliance
   * color.
   * // Optional, defaults to true
   * this // Requires this drive subsystem
   * ));
   * }
   * 
   * public Command followWithEvents(PathPlannerTrajectory path, Map<String,
   * Command> eventMap, boolean isFirstPath) {
   * FollowPathWithEvents command = new FollowPathWithEvents(
   * this.followTrajectoryCommand(path, isFirstPath),
   * path.getMarkers(),
   * eventMap);
   * return command;
   * }
   */

}