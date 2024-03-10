// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.pivotArm.PivotSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.DriveControls.*;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final GroundIntake groundIntake = new GroundIntake();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final Shooter shooter = new Shooter();
  private final PivotSubsystem pivot = new PivotSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriveControls.configureControls();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(DriveControls.DRIVE_FORWARD.getAsDouble(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriveControls.DRIVE_STRAFE.getAsDouble(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriveControls.DRIVE_ROTATE.getAsDouble(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));

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
        shooter.runVoltage(11)
        .alongWith(
            new WaitCommand(0.5)
            .andThen(intake.manualCommand(-IntakeConstants.INTAKE_OUT_VOLTAGE))
        )
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
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
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

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
