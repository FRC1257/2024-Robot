// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import org.littletonrobotics.junction.AutoLogOutput;
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
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.groundIntake.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.pivotArm.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.autonomous.AutoChooser;
import frc.robot.util.autonomous.MakeAutos;
import frc.robot.util.drive.AllianceFlipUtil;
import frc.robot.util.misc.Lookup;
import frc.robot.util.misc.LookupTuner;
import frc.robot.util.note.NoteVisualizer;

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
  private final Indexer indexer;
  private final GroundIntake groundIntake;

  // LEDs
  private final BlinkinLEDController ledController = BlinkinLEDController.getInstance();

  // Mechanisms
  private Mechanism2d mech = new Mechanism2d(3, 3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private LoggedDashboardNumber autoWait = new LoggedDashboardNumber("AutoWait", 0);
  private LoggedDashboardNumber rightShooterVolts = new LoggedDashboardNumber("RightShooter", ShooterConstants.SHOOTER_FULL_VOLTAGE);
  private LoggedDashboardNumber leftShooterVolts = new LoggedDashboardNumber("LeftShooter", ShooterConstants.SHOOTER_FULL_VOLTAGE);

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
        shooter = new Shooter(new ShooterIOSparkMax());
        pivot = new PivotArm(new PivotArmIOSparkMax());
        drive = new Drive(
            new GyroIOReal(),
            new ModuleIOSparkMax(0), // Front Left
            new ModuleIOSparkMax(1), // Front Right
            new ModuleIOSparkMax(2), // Back left
            new ModuleIOSparkMax(3), // Back right
            new VisionIOPhoton());
        indexer = new Indexer(new IndexerIOSparkMax());
        groundIntake = new GroundIntake(new GroundIntakeIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
      case TEST:
        pivot = new PivotArm(new PivotArmIOSim());
        shooter = new Shooter(new ShooterIOSim());
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim());
        indexer = new Indexer(new IndexerIOSim());
        groundIntake = new GroundIntake(new GroundIntakeIOSim());
        break;

      // Replayed robot, disable IO implementations, only reads log files
      default:
        shooter = new Shooter(new ShooterIO() {});
        pivot = new PivotArm(new PivotArmIO() {});
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new VisionIO() {});
        indexer = new Indexer(new IndexerIO() {});
        groundIntake = new GroundIntake(new GroundIntakeIO() {});
        break;
    }

    // Setup Note Visualizer
    NoteVisualizer.setRobotPoseSupplier(drive::getPose, shooter::getLeftSpeedMetersPerSecond,
        shooter::getRightSpeedMetersPerSecond, pivot::getAngle, drive::getFieldVelocity);

    System.out.println("[Init] Setting up Choosers");
    AutoChooser.setupChoosers();
    drive.updateDeadzoneChooser();

    System.out.println("[Init] Setting up Mechanisms");
    
    // Set up robot state manager
    MechanismRoot2d root = mech.getRoot("pivot", 1, 0.5);

    pivot.setMechanism(root.append(pivot.getArmMechanism()));

    // add subsystem mechanisms
    SmartDashboard.putData("Arm Mechanism", mech);

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

    SmartDashboard.putBoolean("Turbo Mode", false);

    // Named Commands
    System.out.println("[Init] Setting up Named Commands");
    
    NamedCommands.registerCommand("Shoot", shootSpeaker().andThen(zeroPosition()));
    NamedCommands.registerCommand("ShootSide", shootSpeakerSide().andThen(zeroPosition()));
    NamedCommands.registerCommand("ShootAnywhere", shootAnywhereAuto());

    NamedCommands.registerCommand("Intake",
        (indexer.IntakeLoopCommand(5).deadlineWith(groundIntake.manualCommand(() -> 5))).deadlineWith(shooter.runVoltage(0)));
    NamedCommands.registerCommand("IntakeWhile", intakeUntilIntaked());

    NamedCommands.registerCommand("Zero", zeroPosition());
    NamedCommands.registerCommand("ZeroPivot", pivot.bringDownCommand());

    NamedCommands.registerCommand("PrepShot", rotateArmSpeaker());
    NamedCommands.registerCommand("PrepShootAnywhere", rotateArmtoSpeakerForever()
                                                              .alongWith(shooter.runVoltageBoth(rightShooterVolts::get, leftShooterVolts::get)));
    NamedCommands.registerCommand("PrepPass", shooter.runVoltage(ShooterConstants.SHOOTER_UNJAM_VOLTAGE).alongWith(indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE)).withTimeout(0.1)
                                                      .andThen(shooter.runVoltage(ShooterConstants.SHOOTER_FULL_VOLTAGE))
                                                      .deadlineWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_PODIUM_ANGLE)));
    NamedCommands.registerCommand("Pass", indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE)
                                                        .alongWith(shooter.runVoltage(ShooterConstants.SHOOTER_FULL_VOLTAGE))
                                                        .alongWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_PODIUM_ANGLE))
                                                        .withTimeout(0.5));

    System.out.println("[Init] Setting up Triggers");
    configureControls();

    // Set up auto routines
    System.out.println("[Init] Setting up Logged Auto Chooser");
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Just Shoot",
        justShootAuto()
    );

    // this is defined later
    autoChooser.addOption("Custom", new InstantCommand());

    // Configure the button bindings
    System.out.println("[Init] Creating Button Bindings");
    configureButtonBindings();

    LookupTuner.setupTuner();
    SmartDashboard.putBoolean("Brake Mode", true);
    SmartDashboard.putBoolean("Set Start Position", false);
    SmartDashboard.putBoolean("ShootSide", false); // TODO comp fix change later
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

    indexer.setDefaultCommand(
        indexer.manualCommand(() -> INTAKE_ROTATE.getAsDouble() * 12));

    groundIntake.setDefaultCommand(
        groundIntake.manualCommand(() -> GROUND_INTAKE_ROTATE.getAsDouble() * 12));

    pivot.setDefaultCommand(pivot.ManualCommand(() -> PIVOT_ROTATE.getAsDouble() * 3));

    shooter.setDefaultCommand(shooter.runVoltage(SHOOTER_SPEED));

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

    // Pivot Commands
    PIVOT_AMP.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_AMP_ANGLE));
    PIVOT_ZERO.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_ARM_INTAKE_ANGLE));
    PIVOT_TO_SPEAKER.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE));
    PIVOT_PODIUM.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_PODIUM_ANGLE));
    PIVOT_ANYWHERE.whileTrue(pivot.PIDCommandForever(this::getAngle));

    // Intake Commands
    INTAKE_IN.whileTrue(indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE));
    INTAKE_OUT.whileTrue(indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE));
    INTAKE_UNTIL_INTAKED.onTrue(intakeUntilIntaked());

    // Ground Intake Commands
    GROUND_INTAKE_IN.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));
    GROUND_INTAKE_OUT.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_OUT_VOLTAGE));

    // Shooter Commands
    SHOOTER_FULL_SEND.whileTrue(shooter.runVoltageBoth(rightShooterVolts::get, leftShooterVolts::get));
    SHOOTER_FULL_SEND_INTAKE.whileTrue(shootNote());
    // Shimmy shimmy
    SHOOTER_UNJAM.whileTrue(
        (indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE / 2)
            .alongWith(shooter.runVoltage(ShooterConstants.SHOOTER_UNJAM_VOLTAGE))));
            
    new Trigger(() -> (int) Timer.getMatchTime() == 30.0).onTrue(getRumbleDriver());
    new Trigger(indexer::isIntaked).onTrue(getRumbleOperator());
    new Trigger(this::isAimedAtSpeaker).onTrue(getRumbleDriver());

    

    if (Constants.tuningMode) {
      SmartDashboard.putData("Pivot Sysid", 
        new SequentialCommandGroup(
          pivot.quasistaticForward(),
          pivot.quasistaticBack(),
          pivot.dynamicForward(),
          pivot.dynamicBack()
        )
      );
    }

    
  }

  public void setPivotPose3d() {
    Logger.recordOutput("PivotPoseThing",
        new Pose3d(
            new Translation3d(0, 0, 0.28),
            new Rotation3d(0, -pivot.getAngle().getRadians(), 0)));

    Logger.recordOutput("PivotPoseSetpoint",
        new Pose3d(
            new Translation3d(0, 0, 0.28),
            new Rotation3d(0, -pivot.getSetpoint().getRadians(), 0)));
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
    AutoChooser.setupChoosers();
    drive.updateDeadzoneChooser();

    if (autoChooser.getSendableChooser().getSelected().equals("Custom")) {
      Command custom = MakeAutos.makeAutoCommand(
        drive,
        this::shootAnywhere,
        this::justShootAuto,
        this::intakeUntilIntaked,
        pivot::bringDownCommand,
        indexer::isIntaked
      );

      if (autoWait.get() > 0) {
        return new WaitCommand(autoWait.get()).andThen(custom);
      }

      return custom;
    }

    if (autoWait.get() > 0) {
      return new WaitCommand(autoWait.get()).andThen(autoChooser.get());
    }

    return autoChooser.get();    
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

  public Command zeroPosition() {
    return pivot.bringDownCommand()
        .deadlineWith(
            indexer.stop()
                .alongWith(shooter.stop())
                .alongWith(groundIntake.stop())).withTimeout(1);
  }

  public Command zeroPositionWhileMoving() {
    return pivot.bringDownCommand()
        .deadlineWith(
            indexer.stop()
                .alongWith(shooter.stop())).withTimeout(1.5);
  }

  public Command shootAmpTrajectory() {
    return drive.goToPose(FieldConstants.ampPose());
  }

  public Command shootAmp() {
    return (rotateArmAmp().andThen(shootNote()));
  }

  public Command altShootAnywhere() {
    return (rotateArmSpeaker()
        .andThen(
            new WaitUntilCommand(this::isPointedAtSpeaker).deadlineWith(rotateArmSpeaker())
                .andThen(shootNote().deadlineWith(rotateArmSpeaker())))) // problem is here, both of these commands can't be
                                                                  // robotContainer
        .deadlineWith(DriveCommands.joystickSpeakerPoint(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE)); 
  }

  public boolean isPointedAtSpeaker() {
    return DriveCommands.pointedAtSpeaker(drive);
  }

  public boolean isAimedAtSpeaker() {
    return DriveCommands.pointedAtSpeaker(drive) && pivot.atSetpoint();
  }

  public Command shootAnywhere() {
    return (new WaitUntilCommand(this::isPointedAtSpeaker).andThen(shootNote()))
              .deadlineWith(lockOnSpeakerFull());
  }

  public Command shootAnywhereAuto() {
    return (new WaitUntilCommand(this::isAimedAtSpeaker).andThen(indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE).withTimeout(1)))
              .deadlineWith(shooter.runVoltageBoth(rightShooterVolts::get, leftShooterVolts::get))
              .deadlineWith(DriveCommands.joystickSpeakerPoint(drive, () -> 0, () -> 0))
              .deadlineWith(rotateArmtoSpeakerForever());
  }

  public Command justShootAuto() {
    return shootSpeaker().onlyIf(DriveCommands::getPivotSideAngle)
            .andThen(shootSpeakerSide().onlyIf(() -> !DriveCommands.getPivotSideAngle()));
  }

  public Command prepShooter() {
    return shooter.runVoltageBoth(rightShooterVolts::get, leftShooterVolts::get);
  }

  public Command shootSpeaker() {
    return (
      rotateArmSpeaker().deadlineWith(prepShooter())
        .andThen(shootNote().deadlineWith(rotateArmSpeaker().repeatedly())));
  }

  public Command shootSpeakerSide() {
    return (
      rotateArmSpeakerSide().deadlineWith(prepShooter())
        .andThen(shootNote().deadlineWith(rotateArmSpeakerSide().repeatedly())));
  }

  public Command rotateArmtoSpeakerForever() {
      return pivot.PIDCommandForever(this::getAngle);
  }

  public Command rotateArmtoTrap() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_TRAP_ANGLE);
  }

  public Command rotateArmSpeaker() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE).withTimeout(PivotArmConstants.PIVOT_MAX_PID_TIME);
  }

  public Command rotateArmSpeakerSide() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_SUBWOOFER_SIDE_ANGLE).withTimeout(PivotArmConstants.PIVOT_MAX_PID_TIME);
  }

  public Command rotateArmAmp() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_AMP_ANGLE);
  }

  public Command shootTrap() {
    return (drive.goToPose(FieldConstants.TrapPose).andThen(rotateArmtoTrap()).andThen(shootNote()));
  }

  public Command lockOnSpeakerFull() {
    return (rotateArmtoSpeakerForever()) // problem is here, both of these commands can't be robotContainer
        .alongWith(DriveCommands.joystickSpeakerPoint(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE));
  }

  public Command shootNote() {
    return shooter.runVoltageBoth(rightShooterVolts::get, leftShooterVolts::get) // run shooter full speed
        .alongWith(
            new SequentialCommandGroup(
              indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE / 2).withTimeout(0.1), // run intake back for 0.1 seconds
              new WaitUntilCommand(0.2),
              indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE) // run intake in to shoot
            ))
        .withTimeout(1.5);//.alongWith(new InstantCommand(() -> {NoteVisualizer.shoot().schedule();})); // run the visualizer
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
  @AutoLogOutput(key = "DistanceAway")
  private double getEstimatedDistance() {
    Transform2d targetTransform = getEstimatedPosition().minus(FieldConstants.speakerPosition());
    return targetTransform.getTranslation().getNorm();
  }

  // Gets RPM based on distance from speaker, taking into account the actual
  // shooting position
  /* private double getRPM() {
    return Lookup.getRPM(getEstimatedDistance());
  } */

  // Gets angle based on distance from speaker, taking into account the actual
  // shooting position
  private double getAngle() {
    double angle = Lookup.getAngle(getEstimatedDistance());
    Logger.recordOutput("ShootAnywhereAngle", angle);
    return angle;
  }

  public void LEDPeriodic() {
    BlinkinLEDController.isEndgame = DriverStation.getMatchTime() <= 30;
    BlinkinLEDController.isEnabled = DriverStation.isEnabled();
    // BlinkinLEDController.noteInIntake = intake.isIntaked();
    BlinkinLEDController.pivotArmDown = pivot.getAngle().getRadians() < (PivotArmConstants.PIVOT_ARM_MIN_ANGLE + Math.PI / 6);
    BlinkinLEDController.shooting = shooter.getLeftSpeedMetersPerSecond() > 5_000;
    ledController.periodic();
  }

  public void disabledPeriodic() {
    LEDPeriodic();
    if (SmartDashboard.getBoolean("Brake Mode", true) != brakeMode) {
      brakeMode = !brakeMode;
      pivot.setBrake(brakeMode);
    }

    if (SmartDashboard.getBoolean("Set Start Position", false)) {
      AutoChooser.setupChoosers();
      drive.updateDeadzoneChooser();
      resetRobotPose(AutoChooser.getStartPose());
      SmartDashboard.putBoolean("Set Start Position", false);
    }

    setPivotPose3d();
    field.setRobotPose(drive.getPose());
    Logger.recordOutput("DriveAimed", DriveCommands.pointedAtSpeaker(drive));
    Logger.recordOutput("PivotAimed", pivot.atSetpoint());
  }

  public Command intakeUntilIntaked(){
    return indexer.IntakeLoopCommand(IndexerConstants.INDEXER_IN_VOLTAGE_WEAK).deadlineWith(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE)).deadlineWith(shooter.runVoltage(0));
  }
  
}