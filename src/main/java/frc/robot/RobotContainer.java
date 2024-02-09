// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;


import static frc.robot.Constants.ShooterConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.drive.*;


//import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.Constants.PivotArm.PivotArmSimConstants;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.GoToPose;
import frc.robot.commands.TurnAngleCommand;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.util.CommandSnailController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.List;


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
  private final Shooter shooter;
  private final PivotArm pivot;
  private Mechanism2d mech = new Mechanism2d(3, 3);
  private Intake intake;
  // Controllers
  private final CommandSnailController driver = new CommandSnailController(0);
  private final CommandSnailController operator = new CommandSnailController(1);

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

        shooter = new Shooter(new ShooterIOSparkMax());
        pivot = new PivotArm(new PivotArmIOSparkMax());
        drive = new Drive(
            new GyroIOReal(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3),
            new VisionIOPhoton());
        intake = new Intake(new IntakeIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
      case TEST:
        pivot = new PivotArm(new PivotArmIOSim());
        shooter = new Shooter(new ShooterIOSim());
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim());
            intake = new Intake(new IntakeIOSim());
        break;

      // Replayed robot, disable IO implementations
      default
        shooter = new Shooter(new ShooterIO(){});
        pivot = new PivotArm(new PivotArmIO() {});
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
        intake = new Intake(new IntakeIO(){});

        break;
    }

    // Set up robot state manager

    MechanismRoot2d root = mech.getRoot("pivot", 1, 0.5);

    pivot.setMechanism(root.append(pivot.getArmMechanism()));

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
    
     //autoChooser.addOption("Spin", new SpinAuto(drive));
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

    operator.getB().onTrue(pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MAX_ANGLE));
    operator.getX().onTrue(pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MIN_ANGLE));


    intake.setDefaultCommand(
        intake.IntakeSpeedCommand(
          () -> operator.getLeftX() * 120
        )
      );

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        2.0));



    shooter.setDefaultCommand(
      shooter.runSpeed(0)
    );

    // cancel trajectory
    driver.getY().onTrue(drive.endTrajectoryCommand());

    operator.a().whileTrue(shooter.runSpeed(ShooterConstants.defaultShooterSpeedRPM));

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