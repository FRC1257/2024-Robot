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

package frc.robot.util.note;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.Supplier;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// from here https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.java
public class NoteVisualizer {
  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static final Transform3d launcherTransform =
      new Transform3d(0.35, 0, 0.8, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));
  private static final double shotSpeed = 5.0; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  private static Supplier<Double> leftSpeed = () -> 10.0;
  private static Supplier<Double> rightSpeed = () -> 10.0;
  private static Supplier<Rotation2d> pivotAngle = () -> Rotation2d.fromDegrees(90.0);

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier, Supplier<Double> leftSpeedSupplier, Supplier<Double> rightSpeedSupplier, Supplier<Rotation2d> pivotAngleSupplier) {
    robotPoseSupplier = supplier;
    leftSpeed = leftSpeedSupplier;
    rightSpeed = rightSpeedSupplier;
    pivotAngle = pivotAngleSupplier;
  }

  public static Command shoot(Drive drive) {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  NoteShot shot = NoteShot.fromRobotInfo(robotPoseSupplier.get(), pivotAngle.get(), leftSpeed.get(), rightSpeed.get(), drive);

                  NotePath path = new NotePath(shot);
                  path.generatePath();

                  double duration = path.getTotalTime();
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  path.samplePathPoint(timer.get())
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(false));
  }
}