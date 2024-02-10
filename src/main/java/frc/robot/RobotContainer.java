// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.TurnAngleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.DriveControls;

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

  // Field
  private final Field2d field;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(
            new GyroIOReal(),
            new ModuleIOSparkMax(0), //Front Left
            new ModuleIOSparkMax(1), //Front Right
            new ModuleIOSparkMax(2), //Back left
            new ModuleIOSparkMax(3), //Back right
            new VisionIOPhoton()); 
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new VisionIO() {
            });
        break;
    }

    // Set up robot state manager

    MechanismRoot2d root = mech.getRoot("pivot", 1, 0.5);
    // add subsystem mechanisms
    SmartDashboard.putData("Arm Mechanism", mech);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });

    DriveControls.configureControls();

    // Set up auto routines
    /*
     * NamedCommands.registerCommand(
     * "Run Flywheel",
     * Commands.startEnd(
     * () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
     * flywheel)
     * .withTimeout(5.0));
     */
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption("Drive Trajectory",
        drive.getAuto("Forward And Spin"));

    autoChooser.addOption("Drive Try Trajectory",
        drive.getAuto("thinger"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //drive.setDefaultCommandRobotRelative
    drive.setDefaultCommand( //change state here
        DriveCommands.joystickDrive(
            drive,
            DriveControls.DRIVE_FORWARD,
            DriveControls.DRIVE_STRAFE,
            DriveControls.DRIVE_ROTATE));

    DriveControls.DRIVE_SPEAKER_AIM.whileTrue(DriveCommands.joystickDriveRobotRelative(
      drive,
      DriveControls.DRIVE_FORWARD,
      DriveControls.DRIVE_STRAFE,
      DriveControls.DRIVE_ROTATE
    ));
            
    DriveControls.DRIVE_SPEAKER_AIM.whileTrue(
      DriveCommands.joystickSpeakerPoint(
          drive,
          DriveControls.DRIVE_FORWARD,
          DriveControls.DRIVE_STRAFE
    ));
    
    DriveControls.DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DriveControls.DRIVE_AMP.onTrue(drive.goToPose(FieldConstants.ampPose));
    DriveControls.DRIVE_SOURCE.onTrue(drive.goToPose(FieldConstants.pickupPose));
    DriveControls.DRIVE_STOP.onTrue(new InstantCommand(drive::stopWithX, drive));

    DriveControls.TURN_90.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(-90)));
    DriveControls.TURN_180.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(180)));

    if (Constants.tuningMode) {
      SmartDashboard.putData("Sysid Dynamic Drive Forward", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Sysid Dynamic Drive Backward", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      SmartDashboard.putData("Sysid Dynamic Turn Forward", drive.turnDynamic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Sysid Dynamic Turn Backward", drive.turnDynamic(SysIdRoutine.Direction.kReverse));

      SmartDashboard.putData("Sysid Quasi Drive Forward", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Sysid Quasi Drive Backward", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      SmartDashboard.putData("Sysid Quasi Turn Forward", drive.turnQuasistatic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Sysid Quasi Turn Backward", drive.turnQuasistatic(SysIdRoutine.Direction.kReverse));
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}