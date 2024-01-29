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

import static frc.robot.Constants.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.Constants.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.kRearRightTurningCanId;

import java.util.Queue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;


import static frc.robot.Constants.ModuleConstants.*;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;



  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: //Front Left
        driveSparkMax = new CANSparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = Rotation2d.fromDegrees(90); // MUST BE CALIBRATED
        break;
      case 1: //Front Right
        driveSparkMax = new CANSparkMax(kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = Rotation2d.fromDegrees(180); // MUST BE CALIBRATED
        break;
      case 2: //Back Left
        driveSparkMax = new CANSparkMax(kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kRearLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = Rotation2d.fromDegrees(180); // MUST BE CALIBRATED
        break;
      case 3: //Back Right
        driveSparkMax = new CANSparkMax(kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kRearRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = Rotation2d.fromDegrees(180); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Restore factory defaults to ensure we don't have any unexpected settings
    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();
    
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnAbsoluteEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turnAbsoluteEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    turnAbsoluteEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // turnAbsoluteEncoder.setZeroOffset(absoluteEncoderOffset.getRadians());

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    
    // Configure Current Limits
    driveSparkMax.setSmartCurrentLimit(kDrivingMotorCurrentLimit);
    turnSparkMax.setSmartCurrentLimit(kTurningMotorCurrentLimit);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    // Configure Brake Modes
    driveSparkMax.setIdleMode(kDrivingMotorIdleMode);
    turnSparkMax.setIdleMode(kTurningMotorIdleMode);

    // Configure Drive Encoder
    driveEncoder.setPosition(0.0);
    driveEncoder.setPositionConversionFactor(kDrivingEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(kDrivingEncoderVelocityFactor);
    
    // Configure Drive PID Controller
    drivePIDController = driveSparkMax.getPIDController();
    drivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turnAbsoluteEncoder.setInverted(isTurnMotorInverted);
    turnAbsoluteEncoder.setPositionConversionFactor(kTurningEncoderPositionFactor);
    turnAbsoluteEncoder.setVelocityConversionFactor(kTurningEncoderVelocityFactor);

    // Configure Turn PID Controller
    turnPIDController = turnSparkMax.getPIDController();

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    turnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // Pass in a sensor (encoder) to use as feedback
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnAbsoluteEncoder);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    // Log things?
    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnAbsoluteEncoder::getPosition);

    // Save our settings
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition = getTurnPosition();
    inputs.turnPosition = getTurnPosition();
    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    drivePIDController.setP(p);
    drivePIDController.setI(i);
    drivePIDController.setD(d);
    drivePIDController.setFF(ff);
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    turnPIDController.setP(p);
    turnPIDController.setI(i);
    turnPIDController.setD(d);
    turnPIDController.setFF(ff);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    drivePIDController.setReference(velocityRadPerSec, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(double angle) {
    turnPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public Rotation2d getTurnPosition() {
    double angle = turnAbsoluteEncoder.getPosition();
    if (angle < 0) {
      angle += 2 * Math.PI;
    }
    if (angle > 2 * Math.PI) {
      angle -= 2*Math.PI;
    }
    return Rotation2d.fromRadians(angle);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public double getTurnPositionError(double angle) {
    return Math.abs(turnAbsoluteEncoder.getPosition() - angle);
  }
}