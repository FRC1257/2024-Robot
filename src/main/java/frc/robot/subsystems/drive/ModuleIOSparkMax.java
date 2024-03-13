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

import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightTurningCanId;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.SparkPIDController;

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

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private final double absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: //Front Left
        m_drivingSparkMax = new CANSparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(kFrontLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kFrontLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 1: //Front Right
        m_drivingSparkMax = new CANSparkMax(kFrontRightDrivingCanId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(kFrontRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kFrontRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 2: //Back Left
        m_drivingSparkMax = new CANSparkMax(kRearLeftDrivingCanId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(kRearLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 3: //Back Right
        m_drivingSparkMax = new CANSparkMax(kRearRightDrivingCanId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(kRearRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = getDrivePosition() / (ModuleConstants.kWheelDiameterMeters / 2);
    inputs.drivePositionMeters = getDrivePosition();
    inputs.driveVelocityMeterPerSec = getVelocityEncoderPosition();
    inputs.driveVelocityRadPerSec = getVelocityEncoderPosition() / (ModuleConstants.kWheelDiameterMeters / 2);
    inputs.driveAppliedVolts = m_drivingSparkMax.getAppliedOutput() * m_drivingSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = m_drivingSparkMax.getOutputCurrent();

    inputs.turnAbsolutePosition = new Rotation2d(getTurnEncoderPosition());
    inputs.turnPosition = new Rotation2d(getTurnEncoderPosition());
    inputs.turnVelocityRadPerSec = getVelocityEncoderPosition();
    inputs.turnAppliedVolts = m_turningSparkMax.getAppliedOutput() * m_turningSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = m_turningSparkMax.getOutputCurrent();

  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    m_drivingPIDController.setP(p);
    m_drivingPIDController.setI(i);
    m_drivingPIDController.setD(d);
    m_drivingPIDController.setFF(ff);
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    m_turningPIDController.setP(p);
    m_turningPIDController.setI(i);
    m_turningPIDController.setD(d);
    m_turningPIDController.setFF(ff);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_drivingPIDController.setReference(velocityRadPerSec, CANSparkMax.ControlType.kVelocity);  }

  @Override
  public void setTurnPosition(double angle) { // radians
    // m_turningPIDController.setReference(angle + absoluteEncoderOffset, CANSparkMax.ControlType.kPosition);
    m_turningPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_drivingSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turningSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_drivingSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turningSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
  
  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }

  @Override
  public double getTurnEncoderPosition() {
    return m_turningEncoder.getPosition();
  }

  @Override
  public double getDrivePosition() {
    return m_drivingEncoder.getPosition();
  }

  @Override
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public double getVelocityEncoderPosition() {
    return m_drivingEncoder.getVelocity();
  }
}