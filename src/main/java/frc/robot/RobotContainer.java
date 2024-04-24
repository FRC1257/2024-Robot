// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.drive.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Mechanisms
  private Mechanism2d mech = new Mechanism2d(3, 3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private LoggedDashboardNumber autoWait = new LoggedDashboardNumber("AutoWait", 0);

  // Field
  private final Field2d field;

  private boolean brakeMode = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("[Init] Creating Subsystems");
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(
            new GyroIOReal(),
            new ModuleIOSparkMax(0), // Front Left
            new ModuleIOSparkMax(1), // Front Right
            new ModuleIOSparkMax(2), // Back left
            new ModuleIOSparkMax(3), // Back right
            new VisionIOPhoton());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
      case TEST:
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim());
        break;

      // Replayed robot, disable IO implementations, only reads log files
      default:
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new VisionIO() {});
        break;
    }

    System.out.println("[Init] Setting up Mechanisms");

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    System.out.println("[Init] Setting up Path Planner Logging");

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
      Logger.recordOutput("PathPlanner/RobotPose", pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
      Logger.recordOutput("PathPlanner/TargetPose", pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
      Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
    });


    System.out.println("[Init] Setting up Triggers");
    configureControls();

    // Set up auto routines
    System.out.println("[Init] Setting up Logged Auto Chooser");
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    System.out.println("[Init] Creating Button Bindings");
    configureButtonBindings();

    SmartDashboard.putBoolean("Brake Mode", true);
    SmartDashboard.putBoolean("Set Start Position", false);
  }

  public void reset() {
    drive.resetYaw();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // default subsystem commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE,
            DRIVE_ROTATE));

    // Drive setting commands
    DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DRIVE_STOP.onTrue(new InstantCommand(() -> {
      drive.stopWithX();
      drive.resetYaw();
    }, drive));

    // Drive Modes
    DRIVE_ROBOT_RELATIVE.whileTrue(DriveCommands.joystickDrive(
        drive,
        DRIVE_FORWARD,
        DRIVE_STRAFE,
        DRIVE_ROTATE));

    DRIVE_SPEAKER_AIM.whileTrue(
        DriveCommands.joystickSpeakerPoint(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE));

    // Drive Angle Locks
    LOCK_BACK.whileTrue(DriveCommands.joystickAnglePoint(
        drive, DRIVE_FORWARD, DRIVE_STRAFE,
        () -> {
          return AllianceFlipUtil.apply(new Rotation2d());
        }
      ));
    LOCK_PICKUP.whileTrue(DriveCommands.joystickAnglePoint(
        drive, DRIVE_FORWARD, DRIVE_STRAFE,
        () -> {
          return AllianceFlipUtil.apply(Rotation2d.fromDegrees(-45));
        }
      ));
    LOCK_PASS.whileTrue(DriveCommands.joystickPasserPoint(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE));
    LOCK_ON_AMP.whileTrue(joystickAmpPoint());

    // Amp Drive Trajectory
    DRIVE_AMP.onTrue(shootAmpTrajectory());

    new Trigger(() -> (int) Timer.getMatchTime() == 30.0).onTrue(getRumbleDriver());

  }

  public void resetRobotPose(Pose2d pose) {
    drive.resetPose(pose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(autoWait.get()).andThen(autoChooser.get());
  }

  /**
   * Drives while continuously facing the amp
   */
  public Command joystickAmpPoint() {
    return DriveCommands.joystickAnglePoint(
      drive,
      DRIVE_FORWARD,
      DRIVE_STRAFE,
      () -> {
        return AllianceFlipUtil.apply(new Rotation2d(Units.degreesToRadians(-90)));
      }
    );
  }

  public Command shootAmpTrajectory() {
    return drive.goToPose(FieldConstants.ampPose());
  }

  public boolean isPointedAtSpeaker() {
    return DriveCommands.pointedAtSpeaker(drive);
  }

  /* // Brings the note forward and back for 0.5 seconds each to center it
  public Command intakeShimmyCommand() {
    return (indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE)
      .alongWith(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE)))
      .withTimeout(ShooterConstants.SHOOTER_SPINUP_TIME)
      .andThen(indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE).withTimeout(ShooterConstants.SHOOTER_SPINUP_TIME));
  } */

  // Returns the estimated transformation over the next tick (The change in
  // position)
  private Transform2d getEstimatedTransform() {
    return new Transform2d(new Translation2d(drive.getFieldVelocity().vxMetersPerSecond * 0.02,
        drive.getFieldVelocity().vyMetersPerSecond * 0.02), new Rotation2d(0.0));
  }

  // Returns the estimated robot position
  private Pose2d getEstimatedPosition() {
    return drive.getPose().plus(getEstimatedTransform().inverse());
  }

  // Returns the distance between the robot's next estimated position and the
  // speaker position
  private double getEstimatedDistance() {
    Transform2d targetTransform = getEstimatedPosition().minus(FieldConstants.speakerPosition());
    Logger.recordOutput("DistanceAway", targetTransform.getTranslation().getNorm());

    return targetTransform.getTranslation().getNorm();
  }

  public void LEDPeriodic() {
    
  }

  public void disabledPeriodic() {
    LEDPeriodic();
    if (SmartDashboard.getBoolean("Brake Mode", true) != brakeMode) {
      brakeMode = !brakeMode;
    }

    if (SmartDashboard.getBoolean("Set Start Position", false)) {
      resetRobotPose(new Pose2d());
      SmartDashboard.putBoolean("Set Start Position", false);
    }

    field.setRobotPose(drive.getPose());
  }

}