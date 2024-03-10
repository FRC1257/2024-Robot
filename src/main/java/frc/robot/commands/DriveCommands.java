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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.shooter.*;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.drive.DriveControls.DRIVE_AMP;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static double slowMode = 
    1;
    // kSlowModeConstant;

  private static PIDController angleController = new PIDController(kTurnSpeakerP, kTurnSpeakerI, kTurnSpeakerD);

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * slowMode, DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDriveRobotRelative(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
    }

    /**
     * Drive robot while pointing at a specific point on the field.
     */
    public static Command joystickSpeakerPoint(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier) {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
            () -> { 
                Pose2d speakerPose = new Pose2d(-0.2, (5 + 6.12)/2, new Rotation2d(0));
                // Apply deadband
                double linearMagnitude =
                    MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode), DEADBAND);
                Rotation2d linearDirection =
                    new Rotation2d(xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
                Transform2d targetTransform = drive.getPose().minus(speakerPose);
                Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                // Rotation2d deltaDirection = drive.getRotation().minus(targetDirection);
                
                double omega = angleController.calculate(drive.getRotation().getRadians(), targetDirection.getRadians());

                // Square values
                linearMagnitude = linearMagnitude * linearMagnitude;
                omega = Math.copySign(omega * omega, omega);

                // Calcaulate new linear velocity
                Translation2d linearVelocity =
                    new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

                // Convert to robot relative speeds & send command
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        drive.getRotation()));
            },
        drive
        );
    }
    // turns robot to speaker from current location
    public static Command turnSpeakerAngle(Drive drive) {
        Pose2d speakerPose = FieldConstants.SpeakerPosition;
        angleController.setTolerance(0.08, 0.01);
            return new FunctionalCommand(
                () -> {
                    Transform2d targetTransform = drive.getPose().minus(speakerPose);
                    Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                    angleController.setSetpoint(targetDirection.getRadians());
                },
                () -> {
                    // defines distance from speaker
                    Transform2d targetTransform = drive.getPose().minus(speakerPose);
                    Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                    double omega = angleController.calculate(drive.getRotation().getRadians(), targetDirection.getRadians());
                    omega = Math.copySign(omega * omega, omega); //no idea why squared
                    // Convert to robot relative speeds and send command
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            0,
                            0,
                            omega * drive.getMaxAngularSpeedRadPerSec(),
                            drive.getRotation()));
                },
                (interrupted) -> {
                    drive.stop();
                },
                () -> angleController.atSetpoint(),
                drive);
        }
       

    public static Command turnToNote(Drive drive) {
        Pose2d notePose = drive.calculateNotePose(drive.getPose(), drive.calculateNoteTranslation());
           angleController.setTolerance(0.08, 0.01);
               return new FunctionalCommand(
                   () -> {
                       Transform2d targetTransform = drive.getPose().minus(notePose);
                       Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                       angleController.setSetpoint(targetDirection.getRadians());
                   },
                   () -> {
                       // defines distance from speaker
                       Transform2d targetTransform = drive.getPose().minus(notePose);
                       Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                       double omega = angleController.calculate(drive.getRotation().getRadians(), targetDirection.getRadians());
                       omega = Math.copySign(omega * omega, omega);
                       // Convert to robot relative speeds and send command
                       drive.runVelocity(
                           ChassisSpeeds.fromFieldRelativeSpeeds(
                               0,
                               0,
                               omega * drive.getMaxAngularSpeedRadPerSec(),
                               drive.getRotation()));
                   },
                   (interrupted) -> {
                       drive.stop();
                   },
                   () -> angleController.atSetpoint(),
                   drive);
    }

    public static Command driveNote(Drive drive) {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
            () -> { 
                Rotation2d targetDirection = drive.getAngleToNote();

                double linearMagnitude = slowMode * targetDirection.getCos();
                double omega = angleController.calculate(targetDirection.getRadians(), 0);

                // Square values
                linearMagnitude = linearMagnitude * linearMagnitude;
                omega = Math.copySign(omega * omega, omega);

                // Convert to robot relative speeds & send command
                drive.runVelocity(
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        linearMagnitude,
                        0,
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        drive.getRotation()
                    )
                );
            },
        drive
        );
    }

    /** 
         * Toggle Slow Mode
         */
    public static void toggleSlowMode() {
      if (slowMode == 1) {
        slowMode = kSlowModeConstant;
      } else {
        slowMode = 1;
      }
    }

    public static boolean pointedAtSpeaker(Drive drive){
        Pose2d speakerPose = FieldConstants.SpeakerPosition;
        Transform2d targetTransform = drive.getPose().minus(speakerPose);
        Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
        
        // Convert to robot relative speeds and send command
        if (Math.abs(drive.getRotation().getDegrees() - targetDirection.getDegrees()) < 1) {
            return true;
        } else {
            return false;
        }
      }

    public static Command driveBackAuto(Drive drive, Shooter shooter, PivotArm pivot, Intake intake){
        return  //(pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE+0.005))
            //  .andThen(
            //      (shooter.runVoltage(11).withTimeout(4)
            //          .alongWith(
            //              new WaitCommand(0.5)
            //              .andThen(intake.manualCommand(-IntakeConstants.INTAKE_OUT_VOLTAGE).withTimeout(2)
            //          )
            //          )).deadlineWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE+0.005)).withTimeout(5)
                
            //      ).andThen(
                    //
                 joystickDrive(drive, () -> 0.6, () -> 0, ()-> 0).withTimeout(1.5);//);
 }
}