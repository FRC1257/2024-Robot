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

import static frc.robot.subsystems.drive.ModuleConstants.kDrivingD;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingFF;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingI;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingP;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningD;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningFF;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningI;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningP;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.setDrivePIDFF(kDrivingP, kDrivingI, kDrivingD, kDrivingFF);
        io.setTurnPIDFF(kTurningP, kTurningI, kTurningD, kTurningFF);
        break;
      case SIM:
        io.setDrivePIDFF(0.1, 0, 0, 0);
        io.setTurnPIDFF(10.0, 0, 0, 0);
        break;
      default:
        io.setDrivePIDFF(0, 0, 0, 0);
        io.setTurnPIDFF(0, 0, 0, 0);
        break;
    }

    setBrakeMode(true);
    m_desiredState.angle = new Rotation2d(io.getTurnEncoderPosition());
    io.resetEncoders();
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
   // if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
     // turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    //}

    // Run closed loop turn control
    /* if (angleSetpoint != null) {
      io.setTurnPosition(angleSetpoint.getRadians());

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        // double adjustSpeedSetpoint = speedSetpoint * Math.cos(io.getTurnPositionError(angleSetpoint.getRadians()));

        // Run drive controller
        // double velocityRadPerSec = adjustSpeedSetpoint;

        io.setDriveVelocity(speedSetpoint);
      } */

  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(io.getVelocityEncoderPosition(),
        new Rotation2d(io.getTurnEncoderPosition() - io.getAbsoluteEncoderOffset()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        io.getDrivePosition(),
        new Rotation2d(io.getTurnEncoderPosition() - io.getAbsoluteEncoderOffset()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(io.getAbsoluteEncoderOffset()));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(io.getTurnEncoderPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    io.setDriveVelocity(optimizedDesiredState.speedMetersPerSecond/* , CANSparkMax.ControlType.kVelocity */);
    io.setTurnPosition(optimizedDesiredState.angle.getRadians()/* , CANSparkMax.ControlType.kPosition */);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    io.resetEncoders();
  }
}