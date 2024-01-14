package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.UPDATE_PERIOD;

public class DriveIOSparkMax implements DriveIO {
  private static final double GEAR_RATIO = 6.0;

  private final CANSparkMax leftLeader;
  private final CANSparkMax rightLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;

  private PIDController anglePIDController;

  private final GyroIOReal gyro;

  public DriveIOSparkMax() {
    leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    rightLeader = new CANSparkMax(2, MotorType.kBrushless);
    leftFollower = new CANSparkMax(3, MotorType.kBrushless);
    rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    configureEncoders();
    configurePID();

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    gyro = GyroIOReal.getInstance();
  }

  // configure all encoder settings and conversion factors
  private void configureEncoders() {
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
    rightEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
    leftEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);
    rightEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.gyroYawRad = gyro.getYawAngle();
    inputs.gyroRollPitchYawRad[0] = gyro.getRollAngle();
    inputs.gyroRollPitchYawRad[1] = gyro.getPitchAngle();
    inputs.gyroRollPitchYawRad[2] = gyro.getYawAngle();
    inputs.timestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public double getLeftPositionMeters() {
    return leftEncoder.getPosition() * leftEncoder.getPositionConversionFactor();
  }

  @Override
  public double getRightPositionMeters() {
    return rightEncoder.getPosition() * rightEncoder.getPositionConversionFactor();
  }

  // configure all PID settings on the motors
  private void configurePID() {
    // configure velocity PID controllers
    leftPIDController = leftLeader.getPIDController();
    rightPIDController = rightLeader.getPIDController();

    leftPIDController.setP(DRIVE_VEL_LEFT_P, DRIVE_VEL_SLOT);
    leftPIDController.setFF(DRIVE_VEL_LEFT_F, DRIVE_VEL_SLOT);
    rightPIDController.setP(DRIVE_VEL_RIGHT_P, DRIVE_VEL_SLOT);
    rightPIDController.setFF(DRIVE_VEL_RIGHT_F, DRIVE_VEL_SLOT);

    // configure turn angle PID controllers
    anglePIDController = new PIDController(DRIVE_ANGLE_PID[0], DRIVE_ANGLE_PID[1], DRIVE_ANGLE_PID[2], UPDATE_PERIOD);
    anglePIDController.setTolerance(DRIVE_ANGLE_TOLERANCE); // TODO Look into velocity tolerance as well
    anglePIDController.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
    leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
    rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
  }

  @Override
  public void zero() {
    GyroIOReal.getInstance().zeroAll();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public double getRobotAngle() {
    return GyroIOReal.getInstance().getRobotAngle();
  }
}
