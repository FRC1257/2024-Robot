// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.DRIVE_AMP;
import static frc.robot.util.drive.DriveControls.DRIVE_FORWARD;
import static frc.robot.util.drive.DriveControls.DRIVE_ROBOT_RELATIVE;
import static frc.robot.util.drive.DriveControls.DRIVE_ROTATE;
import static frc.robot.util.drive.DriveControls.DRIVE_SLOW;
import static frc.robot.util.drive.DriveControls.DRIVE_SOURCE;
import static frc.robot.util.drive.DriveControls.DRIVE_SPEAKER_AIM;
import static frc.robot.util.drive.DriveControls.DRIVE_STOP;
import static frc.robot.util.drive.DriveControls.DRIVE_STRAFE;
import static frc.robot.util.drive.DriveControls.GROUND_INTAKE_IN;
import static frc.robot.util.drive.DriveControls.GROUND_INTAKE_OUT;
import static frc.robot.util.drive.DriveControls.GROUND_INTAKE_ROTATE;
import static frc.robot.util.drive.DriveControls.INTAKE_IN;
import static frc.robot.util.drive.DriveControls.INTAKE_OUT;
import static frc.robot.util.drive.DriveControls.INTAKE_ROTATE;
import static frc.robot.util.drive.DriveControls.LOCK_ON_SPEAKER_FULL;
import static frc.robot.util.drive.DriveControls.PIVOT_AMP;
import static frc.robot.util.drive.DriveControls.PIVOT_HOLD;
import static frc.robot.util.drive.DriveControls.PIVOT_ROTATE;
import static frc.robot.util.drive.DriveControls.PIVOT_TO_SPEAKER;
import static frc.robot.util.drive.DriveControls.PIVOT_ZERO;
import static frc.robot.util.drive.DriveControls.SHOOTER_FIRE_SPEAKER;
import static frc.robot.util.drive.DriveControls.SHOOTER_FULL_SEND;
import static frc.robot.util.drive.DriveControls.SHOOTER_FULL_SEND_INTAKE;
import static frc.robot.util.drive.DriveControls.SHOOTER_SPEED;
import static frc.robot.util.drive.DriveControls.SHOOTER_UNJAM;
import static frc.robot.util.drive.DriveControls.configureControls;
import static frc.robot.util.drive.DriveControls.getRumbleBoth;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.groundIntake.GroundIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeIO;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.autonomous.AutoChooser;
import frc.robot.util.autonomous.MakeAutos;
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
            new GyroIO() {
            },
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
        shooter = new Shooter(new ShooterIO() {
        });
        pivot = new PivotArm(new PivotArmIO() {
        });
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
        indexer = new Indexer(new IndexerIO() {
        });
        groundIntake = new GroundIntake(new GroundIntakeIO() {
        });
        break;
    }

    System.out.println("[Init] Setting up Logs");
    AutoChooser.setupChoosers();

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

    // Named Commands
    // command calling drivng subystem is probably here
    // NamedCommands.registerCommand("Shoot", shootAnywhere());

    // shootSpeaker aims pivot, shoots; zeroPosition then zeros; run after reaching
    // position
    NamedCommands.registerCommand("Shoot", shootSpeaker().andThen(zeroPosition()));
    NamedCommands.registerCommand("ShootAnywhere", shootAnywhere());
    NamedCommands.registerCommand("Intake",
        indexer.IntakeLoopCommand(5).deadlineWith(groundIntake.manualCommand(() -> 5)));
    // Preps pivot arm at correct angle; may want to run as parallel to movement
    NamedCommands.registerCommand("Zero", zeroPosition());
    configureControls();

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
    autoChooser.addOption("driveOutShoot", DriveCommands.driveBackandShooter(drive, pivot, shooter, indexer));
    autoChooser.addOption("drive out", DriveCommands.driveBackAuto(drive));
    autoChooser.addOption("shoot out", DriveCommands.justShooter(pivot, shooter, indexer));

    // this is defined later
    autoChooser.addOption("Custom", new InstantCommand());

    // autoChooser.addOption("Spin", new SpinAuto(drive));
    // Configure the button bindings
    System.out.println("[Init] Creating Button Bindings");
    configureButtonBindings();

    LookupTuner.setupTuner();
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
    // drive.setDefaultCommandRobotRelative
    drive.setDefaultCommand( // change state here
        DriveCommands.joystickDrive(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE,
            DRIVE_ROTATE));

    indexer.setDefaultCommand(
        indexer.manualCommand(
            () -> INTAKE_ROTATE.getAsDouble() * 12));

    groundIntake.setDefaultCommand(
        groundIntake.manualCommand(
            () -> GROUND_INTAKE_ROTATE.getAsDouble() * 12));

    pivot.setDefaultCommand(
        pivot.ManualCommand(() -> PIVOT_ROTATE.getAsDouble() * 2));
    // pivot.setDefaultCommand(
    // pivot.PIDCommandForever(PIVOT_PID_ROTATE)
    // );

    shooter.setDefaultCommand(
        // shooter.runPIDSpeed(0)
        shooter.runVoltage(SHOOTER_SPEED));

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

    DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DRIVE_AMP.onTrue(drive.goToPose(FieldConstants.ampPose()));
    DRIVE_SOURCE.onTrue(drive.goToPose(FieldConstants.pickupPose()));
    DRIVE_STOP.onTrue(new InstantCommand(() -> {
      drive.stopWithX();
      drive.resetYaw();
    }, drive));

    // TURN_90.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(-90)));
    // TURN_180.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(180)));

    // Operator controls
    PIVOT_AMP.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_AMP_ANGLE));
    PIVOT_ZERO.whileTrue(pivot.PIDCommandForever(0));
    PIVOT_TO_SPEAKER.whileTrue(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE));
    PIVOT_HOLD.whileTrue(pivot.PIDHoldCommand());
    LOCK_ON_SPEAKER_FULL.whileTrue(lockOnSpeakerFull());

    NoteVisualizer.setRobotPoseSupplier(drive::getPose, shooter::getLeftSpeedMetersPerSecond,
        shooter::getRightSpeedMetersPerSecond, pivot::getAngle);

    INTAKE_IN.whileTrue(indexer.manualCommand(IndexerConstants.INDEXER_IN_VOLTAGE));
    INTAKE_OUT.whileTrue(indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE));

    GROUND_INTAKE_IN.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));
    GROUND_INTAKE_OUT.whileTrue(groundIntake.manualCommand(GroundIntakeConstants.GROUND_INTAKE_OUT_VOLTAGE));

    // TODO using voltage mode for now but later speed PID
    SHOOTER_FULL_SEND.whileTrue(shooter.runVoltage(11));
    SHOOTER_FULL_SEND_INTAKE.whileTrue(
        shooter.runVoltage(11)
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(indexer.manualCommand(-IndexerConstants.INDEXER_OUT_VOLTAGE))));

    SHOOTER_UNJAM.whileTrue(
        (indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE / 2)
            .alongWith(shooter.runVoltage(-0.5))));

    // NoteVisualizer.setRobotPoseSupplier(drive::getPose, () -> 10.0, () -> 10.0,
    // pivot::getAngle);
    SHOOTER_FIRE_SPEAKER.onTrue(shootAnywhere());
    // SHOOTER_SHOOT.onTrue(shootNote());
    // SHOOTER_PREP.whileTrue(shooter.runPIDSpeed(ShooterConstants.defaultShooterSpeedRPM));

    new Trigger(() -> (int) Timer.getMatchTime() == 90.0).onTrue(getRumbleBoth());
    // new Trigger(() -> intake.isIntaked()).onTrue(getRumbleBoth());

    // if (Constants.tuningMode) {
    // SmartDashboard.putData("Sysid Dynamic Drive Forward",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // SmartDashboard.putData("Sysid Dynamic Drive Backward",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // SmartDashboard.putData("Sysid Dynamic Turn Forward",
    // drive.turnDynamic(SysIdRoutine.Direction.kForward));
    // SmartDashboard.putData("Sysid Dynamic Turn Backward",
    // drive.turnDynamic(SysIdRoutine.Direction.kReverse));

    // SmartDashboard.putData("Sysid Quasi Drive Forward",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // SmartDashboard.putData("Sysid Quasi Drive Backward",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // SmartDashboard.putData("Sysid Quasi Turn Forward",
    // drive.turnQuasistatic(SysIdRoutine.Direction.kForward));
    // SmartDashboard.putData("Sysid Quasi Turn Backward",
    // drive.turnQuasistatic(SysIdRoutine.Direction.kReverse));
    // }

  }

  public void setPivotPose3d() {
    Logger.recordOutput("PivotPoseThing",
        new Pose3d(
            new Translation3d(0, 0, 0.28),
            new Rotation3d(0, -pivot.getAngle().getRadians(), 0)));
  }

  public void setRobotPose(Pose2d pose) {
    drive.setPose(pose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoChooser.setupChoosers();
    if (autoChooser.getSendableChooser().getSelected().equals("Custom")) {
      return MakeAutos.makeAutoCommand(
          drive,
          this::shootAnywhere,
          () -> {
            return groundIntake.manualCommand(() -> 2).alongWith(indexer.manualCommand(2));
          },
          () -> {
            // use a vision command later
            return new InstantCommand();
          },
          this::zeroPositionWhileMoving);
    }
    return autoChooser.get();
  }

  public Command zeroPosition() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_ARM_MIN_ANGLE)
        .deadlineWith(
            indexer.stop()
                .alongWith(shooter.stop())
                .alongWith(groundIntake.stop()));
  }

  public Command zeroPositionWhileMoving() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_ARM_MIN_ANGLE)
        .deadlineWith(
            indexer.stop()
                .alongWith(shooter.stop()));
  }

  public Command shootAmpTrajectory() {
    return drive.pathfindToTrajectory(PathPlannerPath.fromPathFile("amp score")).andThen(shootAmp());
  }

  public Command shootAmp() {
    return (rotateArmAmp().andThen(shootNote()));
  }

  public Command altShootAnywhere() {
    return (rotateArm()
        .andThen(
            new WaitUntilCommand(this::isPointedAtSpeaker).deadlineWith(rotateArm())
                .andThen(shootNote().deadlineWith(rotateArm())))) // problem is here, both of these commands can't be
                                                                  // robotContainer
        .deadlineWith(DriveCommands.joystickSpeakerPoint(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE)); // .andThen(zeroShooter())
    // the rotate arm method just keeps going, I don't know what's wrong with it
    // Maybe it's the shooter setRPM?
  }

  public boolean isPointedAtSpeaker() {
    return DriveCommands.pointedAtSpeaker(drive);
  }

  public Command shootAnywhere() {
    return (new WaitUntilCommand(this::isPointedAtSpeaker).andThen(shootNote()))
              .deadlineWith(lockOnSpeakerFull());
  }

  public Command shootSpeaker() {
    return (rotateArm().andThen(shootNote()));
  }

  public Command rotateArmtoSpeakerForever() {
    return pivot.PIDCommandForever(this::getAngle);
  }

  public Command rotateArmtoTrap() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_TRAP_ANGLE);
  }

  public Command rotateArm() {
    return pivot.PIDCommand(this::getAngle);
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

  public Command shootSubwoofer() {
    Logger.recordOutput("DistanceAway", getEstimatedDistance());
    return (shooter.runVoltage(11).withTimeout(4)
        .alongWith(
            new WaitCommand(0.5)
                .andThen(indexer.manualCommand(-IndexerConstants.INDEXER_OUT_VOLTAGE).withTimeout(2)))
        .deadlineWith(pivot.PIDCommandForever(PivotArmConstants.PIVOT_SUBWOOFER_ANGLE + 0.005))

    );
  }

  public Command shootNote() {
    return shooter.runVoltage(11)
        .alongWith(
            new WaitCommand(1)
                .andThen(indexer.manualCommand(IndexerConstants.INDEXER_OUT_VOLTAGE)))
        .withTimeout(2);
    // figure out why the shooter is so weaksauce
    // it's only shooting it out fast when I mash the button
    // probably has to do with the getRPM method

    // .alongWith(NoteVisualizer.shoot(drive)));//);
  }

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
    Transform2d targetTransform = getEstimatedPosition().minus(FieldConstants.SpeakerPosition);
    return targetTransform.getTranslation().getNorm();
  }

  // Gets RPM based on distance from speaker, taking into account the actual
  // shooting position
  private double getRPM() {
    return Lookup.getRPM(getEstimatedDistance());
    // return 100000;
    // with a comically high speed it keeps running the point arm command but can't
    // run the shooter command
    // will have to look into this later
  }

  // Gets angle based on distance from speaker, taking into account the actual
  // shooting position
  private double getAngle() {
    return getGeneralAngle(FieldConstants.speakerPosition3D());
  }

  private double getGeneralAngle(Pose3d target) {
    double armLength = PivotArmConstants.PivotArmSimConstants.kArmLength;
    double height = Units.inchesToMeters(target.getZ());
    Transform2d targetTransform = drive.getPose().minus(target.toPose2d());
    double targetDistance = targetTransform.getTranslation().getNorm();
    double angle = Math.PI - (Math.acos(armLength / Math.sqrt(Math.pow(targetDistance, 2) + Math.pow(height, 2)))
        + Math.atan(height / targetDistance));
    Logger.recordOutput("Calculated General Angle", angle);
    return angle;
  }

  public void LEDPeriodic() {
    BlinkinLEDController.isEndgame = DriverStation.getMatchTime() <= 30;
    BlinkinLEDController.isEnabled = DriverStation.isEnabled();
    // BlinkinLEDController.noteInIntake = intake.isIntaked();
    BlinkinLEDController.pivotArmDown = pivot.getAngle()
        .getRadians() < (PivotArmConstants.PIVOT_ARM_MIN_ANGLE + Math.PI / 6);
    BlinkinLEDController.shooting = shooter.getLeftSpeedMetersPerSecond() > 10_000;
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
      setRobotPose(AutoChooser.getStartPose());
      SmartDashboard.putBoolean("Set Start Position", false);
    }

    setPivotPose3d();
  }
}