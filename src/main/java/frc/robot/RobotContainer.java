// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PivotArm.PIVOT_ARM_MIN_ANGLE;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.TurnAngleCommand;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.groundIntake.GroundIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeIO;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.groundIntake.GroundIntake;
import frc.robot.subsystems.groundIntake.GroundIntakeIO;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.pivotArm.PivotArm;
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
import frc.robot.util.AutoChooser;
import frc.robot.util.DriveControls;
import frc.robot.util.Lookup;
import frc.robot.util.LookupTuner;
import frc.robot.util.MakeAutos;
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
  private final Intake intake;
  private final GroundIntake groundIntake;

  // LEDs
  private final BlinkinLEDController ledController = BlinkinLEDController.getInstance();

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
        intake = new Intake(new IntakeIOSparkMax());
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
        intake = new Intake(new IntakeIOSim());
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
        intake = new Intake(new IntakeIO() {
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
    NamedCommands.registerCommand("Shoot", shootAnywhere());
    NamedCommands.registerCommand("Intake", intake.IntakeLoopCommand(3).deadlineWith(groundIntake.GroundIntakeManualCommand(() -> 2)));
    NamedCommands.registerCommand("PrepShoot", prepShoot());
    NamedCommands.registerCommand("Zero", zeroPosition());
    NamedCommands.registerCommand("AmpShooter", setAmpShooterSpeed());
    NamedCommands.registerCommand("AmpAngle", ampPivotAngle());
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

    // this is defined later
    autoChooser.addOption("Custom", new InstantCommand());


    // autoChooser.addOption("Spin", new SpinAuto(drive));
    // Configure the button bindings
    System.out.println("[Init] Creating Button Bindings");
    configureButtonBindings();

    LookupTuner.setupTuner();
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
            DriveControls.DRIVE_FORWARD,
            DriveControls.DRIVE_STRAFE,
            DriveControls.DRIVE_ROTATE));

    intake.setDefaultCommand(
        intake.IntakeSpeedCommand(
            DriveControls.INTAKE_ROTATE));

    groundIntake.setDefaultCommand(
        groundIntake.GroundIntakeSpeedCommand(
            DriveControls.GROUND_INTAKE_ROTATE));

    pivot.setDefaultCommand(
        pivot.ManualCommand(DriveControls.PIVOT_ROTATE));

    shooter.setDefaultCommand(
        shooter.runSpeed(0));

    DriveControls.DRIVE_TOGGLE_ROBOT_RELATIVE.whileTrue(DriveCommands.joystickDriveRobotRelative(
        drive,
        DriveControls.DRIVE_FORWARD,
        DriveControls.DRIVE_STRAFE,
        DriveControls.DRIVE_ROTATE));

    DriveControls.DRIVE_SPEAKER_AIM.whileTrue(
        DriveCommands.joystickSpeakerPoint(
            drive,
            DriveControls.DRIVE_FORWARD,
            DriveControls.DRIVE_STRAFE));

    DriveControls.DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DriveControls.DRIVE_AMP.onTrue(drive.goToPose(FieldConstants.ampPose()));
    DriveControls.DRIVE_SOURCE.onTrue(drive.goToPose(FieldConstants.pickupPose()));
    DriveControls.DRIVE_STOP.onTrue(new InstantCommand(drive::stopWithX, drive));

    DriveControls.TURN_90.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(-90)));
    DriveControls.TURN_180.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(180)));

    // Operator controls
    DriveControls.PIVOT_AMP.onTrue(pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MAX_ANGLE));
    DriveControls.PIVOT_ZERO.onTrue(zeroPosition());

   // NoteVisualizer.setRobotPoseSupplier(drive::getPose, shooter::getLeftSpeedMetersPerSecond, shooter::getRightSpeedMetersPerSecond, pivot::getAngle);
    NoteVisualizer.setRobotPoseSupplier(drive::getPose, () -> 10.0, () -> 10.0, pivot::getAngle);
    DriveControls.SHOOTER_FIRE_SPEAKER.onTrue(shootAnywhere());
    DriveControls.SHOOTER_SHOOT.onTrue(shootNote());
    DriveControls.SHOOTER_PREP.whileTrue(shooter.runSpeed(ShooterConstants.defaultShooterSpeedRPM));

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

  public void setPivotPose3d() {
    Pose2d armPose = drive.getPose().plus(new Transform2d(
        new Translation2d(0.098, drive.getRotation().plus(Rotation2d.fromDegrees(180))), new Rotation2d()));

    Rotation3d rotation = new Rotation3d(0, pivot.getAngle().getRadians(),
        armPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians());
    Translation3d translation = new Translation3d(armPose.getTranslation().getX(), armPose.getTranslation().getY(),
        0.28);
    Pose3d pose = new Pose3d(translation, rotation);
    Logger.recordOutput("PivotPoseThing",
        new Pose3d(
            new Translation3d(0, 0, 0.28),
            new Rotation3d(0, -pivot.getAngle().getRadians(), 0)));
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
          return intake.IntakeManualCommand(() -> 2);
        }, 
        () -> {
          // use a vision command later
          return intake.IntakeLoopCommand(8).withTimeout(1);
        }
      );
    }
    return autoChooser.get();
  }

  public Command zeroPosition() {
    return pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MIN_ANGLE)
        .alongWith(intake.stop())
        .alongWith(shooter.stop())
        .alongWith(groundIntake.stop());
  }

  public Command ampPivotAngle() {
    return pivot.PIDCommand(Units.degreesToRadians(90))
        .alongWith(intake.stop())
        .alongWith(shooter.stop())
        .alongWith(groundIntake.stop());
  }

  public Command setAmpShooterSpeed() {
    return new FunctionalCommand(
        () -> {
          shooter.setRPM(ShooterConstants.defaultShooterSpeedRPM, ShooterConstants.defaultShooterSpeedRPM); // placeholder speed
        },
        () -> {
          shooter.setRPM(ShooterConstants.defaultShooterSpeedRPM, ShooterConstants.defaultShooterSpeedRPM); // placeholder speed
        },
        (interrupted) -> {
          if (!interrupted) return;

          shooter.stop();
        },
        () -> {
          return shooter.atSetpoint();
        },
        shooter
    );
  }

  public Command ampAngleAndShoot() {
    return ampPivotAngle().andThen(shootAmp());
  }

  public Command shootAmp() {
    return drive.pathfindToTrajectory(PathPlannerPath.fromPathFile("amp score"));
  }
  
  public Command shootAnywhere() {
    // implement this later using swerve to turn to desired target
    // move pivot arm
    // and calculate the speed required to shoot
    /* if (DriveCommands.pointedAtSpeaker(drive)){
       return rotateArm().andThen(shoot());
    } else {
    return DriveCommands.turnSpeakerAngle(drive).alongWith(rotateArm()).andThen(shoot()); */

    // return DriveCommands.turnSpeakerAngle(drive).onlyIf(() -> !DriveCommands.pointedAtSpeaker(drive)).alongWith(rotateArm()).andThen(shoot());
    return (rotateArm().alongWith(shoot())).deadlineWith(DriveCommands.joystickSpeakerPoint(
        drive,
        DriveControls.DRIVE_FORWARD,
        DriveControls.DRIVE_STRAFE
      )
    );
  }

  public Command rotateArm(){
    return new FunctionalCommand(
          () -> {},
          () -> {
            pivot.setPID(getAngle());
            pivot.runPID();
            shooter.setRPM(getRPM(), getRPM());
          },
        (interrupted) -> {
          if (!interrupted) return;

          shooter.stop();
          pivot.stop();
        },
        () -> {
          return pivot.atSetpoint() && shooter.atSetpoint();
        },
        shooter, pivot
    );
}

public Command shoot() {
  if(pivot.atSetpoint() == true) {
    return intake.EjectLoopCommand(2).deadlineWith(shooter.runSpeed(() -> getRPM()).alongWith(pivot.PIDCommand(() -> getAngle())).alongWith(NoteVisualizer.shoot(drive)));
  } else {
    return null;
  }
}


  public Command shootNote() {
    return new FunctionalCommand(
        () -> {},
        () -> { 

       
            shooter.setRPM(getRPM(), getRPM());
            //shooter.setRPM(1000, 1000);
          },
        (interrupted) -> {
          if (!interrupted) return;

          shooter.stop();
        },
        () -> {
          return shooter.atSetpoint();
        },
        shooter
    ).andThen(
      intake.EjectLoopCommand(2).deadlineWith(shooter.runSpeed(() -> getRPM()).alongWith(NoteVisualizer.shoot(drive)))
    );
  }
  // Returns the estimated transformation over the next tick (The change in position)
  private Transform2d getEstimatedTransform() {
    return new Transform2d(new Translation2d(drive.getFieldVelocity().vxMetersPerSecond * 0.02, drive.getFieldVelocity().vyMetersPerSecond * 0.02), new Rotation2d(0.0));
  }
  //Returns the estimated robot position
  private Pose2d getEstimatedPosition() {
    return drive.getPose().plus(getEstimatedTransform().inverse());
  }
  //Returns the distance between the robot's next estimated position and the speaker position
  private double getEstimatedDistance() {
    Transform2d targetTransform = getEstimatedPosition().minus(FieldConstants.SpeakerPosition);
    return targetTransform.getTranslation().getNorm();
  }
    // Gets RPM based on distance from speaker, taking into account the actual shooting position
  private double getRPM() {
    return Lookup.getRPM(getEstimatedDistance());
  }
  // Gets angle based on distance from speaker, taking into account the actual shooting position
  private double getAngle() {
    return Lookup.getAngle(getEstimatedDistance());
  }

  public Command prepShoot() {
    // implement this later using swerve to turn to desired target
    // move pivot arm
    // and calculate the speed required to shoot
    return new InstantCommand();
  }

  public void LEDPeriodic() {
    BlinkinLEDController.isEndgame = DriverStation.getMatchTime() <= 30;
    BlinkinLEDController.isEnabled = DriverStation.isEnabled();
    BlinkinLEDController.noteInIntake = intake.isIntaked();
    BlinkinLEDController.pivotArmDown = pivot.getAngle().getRadians() < (PIVOT_ARM_MIN_ANGLE + Math.PI / 6);
    BlinkinLEDController.shooting = shooter.getLeftCharacterizationVelocity() > 100;
    ledController.periodic();
  }
}