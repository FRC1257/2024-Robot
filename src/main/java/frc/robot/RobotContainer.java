// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.DriveControls.*;
import static frc.robot.DriveControls.getRumbleBoth;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.groundIntake.GroundIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivotArm.Pivot;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.autonomous.MakeAutos;

import edu.wpi.first.math.geometry.Pose2d;


import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drive m_robotDrive = new Drive();
  private final GroundIntake groundIntake = new GroundIntake();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Pivot pivot = new Pivot();

    private final SendableChooser<Command> autoChooser;


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriveControls.configureControls();
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Custom", new InstantCommand().withName("Custom"));
    autoChooser.addOption("Leave", driveOut());

    autoChooser.addOption("ShootAndLeave", shoot().andThen(driveOut()));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        DriveCommands.joystickDrive(m_robotDrive, DRIVE_FORWARD,DRIVE_STRAFE, DRIVE_ROTATE)

    );

    DriveControls.DRIVE_TOGGLE_ROBOT_RELATIVE.whileTrue(DriveCommands.joystickDriveRobotRelative(m_robotDrive, DRIVE_FORWARD,DRIVE_STRAFE, DRIVE_ROTATE));

    intake.setDefaultCommand(
        intake.manualCommand(
            () -> DriveControls.INTAKE_ROTATE.getAsDouble() * 12));
        
    groundIntake.setDefaultCommand(
        groundIntake.manualCommand(
            () -> DriveControls.GROUND_INTAKE_ROTATE.getAsDouble() * 12));
        
    pivot.setDefaultCommand(
        pivot.ManualCommand(() -> DriveControls.PIVOT_ROTATE.getAsDouble() * 2));

    shooter.setDefaultCommand(
        shooter.runVoltage(SHOOTER_SPEED));

    DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));


    DRIVE_STOP.onTrue(new InstantCommand(() -> {
      //m_robotDrive.stopWithX();
      m_robotDrive.resetYaw();
    }, m_robotDrive));

    // Operator controls
    PIVOT_AMP.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_AMP_ANGLE));
    PIVOT_ZERO.whileTrue(pivot.PIDCommandForever(0));
    PIVOT_TO_SPEAKER.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE));
    PIVOT_HOLD.whileTrue(pivot.PIDHoldCommand());


    INTAKE_IN.whileTrue(intake.manualCommand(IntakeConstants.INTAKE_IN_VOLTAGE));
    INTAKE_OUT.whileTrue(intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE));

    GROUND_INTAKE_IN.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));
    GROUND_INTAKE_OUT.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_OUT_VOLTAGE));

    // TODO using voltage mode for now but later speed PID
    SHOOTER_FULL_SEND.whileTrue(shooter.runVoltage(11));
    SHOOTER_FULL_SEND_INTAKE.whileTrue(
      shooter.runVoltage(11)
        .alongWith(
          new WaitCommand(0.5)
            .andThen(intake.manualCommand(-IntakeConstants.INTAKE_OUT_VOLTAGE)
        )
    ));

    SHOOTER_UNJAM.whileTrue(
      (intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE/2)
        .alongWith(shooter.runVoltage(-0.5)))
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Shoot", shootSpeaker().andThen(zeroPosition()));
    NamedCommands.registerCommand("Intake",
        intake.manualCommand(IntakeConstants.INTAKE_IN_VOLTAGE).deadlineWith(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE)));
    // Preps pivot arm at correct angle; may want to run as parallel to movement
    NamedCommands.registerCommand("PrepShoot", prepShoot());
    NamedCommands.registerCommand("Zero", zeroPosition());
    // NamedCommands.registerCommand("AmpShooter", setAmpShooterSpeed());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // Operator controls
    PIVOT_AMP.whileTrue(pivot.PIDCommand(PivotArmConstants.PIVOT_AMP_ANGLE));
    PIVOT_ZERO.whileTrue(pivot.PIDCommand(0));
    PIVOT_TO_SPEAKER.whileTrue(pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE));
    PIVOT_HOLD.whileTrue(pivot.PIDHoldCommand());
    // LOCK_ON_SPEAKER_FULL.whileTrue(lockOnSpeakerFull());

    INTAKE_IN.whileTrue(intake.manualCommand(IntakeConstants.INTAKE_IN_VOLTAGE));
    INTAKE_OUT.whileTrue(intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE));

    GROUND_INTAKE_IN.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));
    GROUND_INTAKE_OUT.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_OUT_VOLTAGE));

    // TODO using voltage mode for now but later speed PID
    SHOOTER_FULL_SEND.whileTrue(shooter.runVoltage(11));
    SHOOTER_FULL_SEND_INTAKE.whileTrue(
        shoot()
    );
    SHOOTER_FIRE_AMP.whileTrue(
      shooter.runVoltage(5)
        .alongWith(
          new WaitCommand(1)
            .andThen(intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE)
        )
    ));
    SHOOTER_UNJAM.onTrue(
      (intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE)
        .alongWith(shooter.runVoltage(-1)))
        .withTimeout(IntakeConstants.SHOOTER_UNJAM_TIME)
    );

    new Trigger(() -> (int) Timer.getMatchTime() == 90.0).onTrue(getRumbleBoth());
  }

  public Command shoot() {
    return shooter.runVoltage(11)
        .alongWith(
            new WaitCommand(0.5)
            .andThen(intake.manualCommand(-IntakeConstants.INTAKE_OUT_VOLTAGE))
        );
  }

  public Command intake() {
    return intake.manualCommand(IntakeConstants.INTAKE_IN_VOLTAGE);
  }

  public Command zeroPosition() {
    return pivot.PIDCommand(0).alongWith(intake.manualCommand(0)).alongWith(groundIntake.manualCommand(0));
  }

  public Command shootSpeaker() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE).andThen(
      (
        shooter.runVoltage(11)
          .alongWith(
            new WaitCommand(0.5)
            .andThen(intake.manualCommand(IntakeConstants.INTAKE_OUT_VOLTAGE))
          ).withTimeout(2)
      ).deadlineWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE))
    ).withTimeout(5);
  }

  public Command prepShoot() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE).andThen(
      (
        shooter.runVoltage(11).withTimeout(2)
      ).deadlineWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE))
    );
  }

  public Command intakeWhile() {
    // meant to intake with vision
    return new InstantCommand();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto = autoChooser.getSelected();

    if (auto.getName().equals("Custom")) {
        return MakeAutos.makeAutoCommand(
            m_robotDrive, 
            this::shoot, 
            this::intake, 
            this::intakeWhile, 
            () -> pivot.PIDCommand(0)
        );
    }

    return auto;
    // Create config for trajectory

  }

  public Command driveOut() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        frc.robot.subsystems.drive.DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
