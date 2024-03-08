// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeIO;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.groundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmConstants;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.autonomous.AutoChooser;
import frc.robot.util.autonomous.MakeAutos;
import frc.robot.util.drive.DriveControls;
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
        intake = new Intake(new IntakeIOSparkMax());
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
        intake = new Intake(new IntakeIOSim());
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
          new VisionIO() {}
        );
        intake = new Intake(new IntakeIO() {});
        groundIntake = new GroundIntake(new GroundIntakeIO() {});
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
    NamedCommands.registerCommand("Shoot", shootSpeaker());
    NamedCommands.registerCommand("Intake",
        intake.IntakeLoopCommand(3).deadlineWith(groundIntake.GroundIntakeManualCommand(() -> 2)));
    NamedCommands.registerCommand("PrepShoot", prepShoot());
    NamedCommands.registerCommand("Zero", zeroPosition());
    NamedCommands.registerCommand("AmpShooter", setAmpShooterSpeed());
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
    SmartDashboard.putBoolean("Brake Mode", true);
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
        // intake.IntakeSpeedCommand(
        // DriveControls.INTAKE_ROTATE));
        intake.IntakeManualCommand(
            () -> DriveControls.INTAKE_ROTATE.getAsDouble() * 12));
    // banished to no PID command

    groundIntake.setDefaultCommand(
        groundIntake.GroundIntakeManualCommand(
            () -> DriveControls.GROUND_INTAKE_ROTATE.getAsDouble() * 12));

    // pivot.setDefaultCommand(
    //     pivot.ManualCommand(() -> DriveControls.PIVOT_ROTATE.getAsDouble() * 2));
    pivot.setDefaultCommand(
      pivot.PIDCommandForever(DriveControls.PIVOT_PID_ROTATE)
    );

    shooter.setDefaultCommand(
        // shooter.runPIDSpeed(0)
        shooter.runVoltage(DriveControls.SHOOTER_SPEED));

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

    // DriveControls.DRIVE_NOTE_GOTO.whileTrue(drive.goToNote());

    DriveControls.DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DriveControls.DRIVE_AMP.onTrue(drive.goToPose(FieldConstants.ampPose()));
    DriveControls.DRIVE_SOURCE.onTrue(drive.goToPose(FieldConstants.pickupPose()));
    DriveControls.DRIVE_STOP.onTrue(new InstantCommand(drive::stopWithX, drive));

    DriveControls.TURN_90.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(-90)));
    DriveControls.TURN_180.onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(180)));

    // Operator controls
    DriveControls.PIVOT_AMP.onTrue(pivot.PIDCommand(PivotArmConstants.PIVOT_AMP_ANGLE));
    DriveControls.PIVOT_ZERO.onTrue(pivot.PIDCommand(0));
    //DriveControls.LOCK_ON_SPEAKER_FULL.whileTrue(lockOnSpeakerFull());

    NoteVisualizer.setRobotPoseSupplier(drive::getPose, shooter::getLeftSpeedMetersPerSecond,
        shooter::getRightSpeedMetersPerSecond, pivot::getAngle);

    DriveControls.INTAKE_IN.whileTrue(intake.IntakeManualCommand(() -> IntakeConstants.INTAKE_IN_VOLTAGE));
    DriveControls.INTAKE_OUT.whileTrue(intake.IntakeManualCommand(() -> -IntakeConstants.INTAKE_IN_VOLTAGE));

    DriveControls.GROUND_INTAKE_IN.whileTrue(groundIntake.GroundIntakeManualCommand(() -> GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));
    DriveControls.GROUND_INTAKE_OUT.whileTrue(groundIntake.GroundIntakeManualCommand(() -> -GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE));

    DriveControls.SHOOTER_FULL_SEND.whileTrue(shooter.runVoltage(() -> 11));

    // NoteVisualizer.setRobotPoseSupplier(drive::getPose, () -> 10.0, () -> 10.0,
    // pivot::getAngle);
    DriveControls.SHOOTER_FIRE_SPEAKER.onTrue(shootAnywhere());
    // DriveControls.SHOOTER_SHOOT.onTrue(shootNote());
    // DriveControls.SHOOTER_PREP.whileTrue(shooter.runPIDSpeed(ShooterConstants.defaultShooterSpeedRPM));

    new Trigger(() -> Timer.getMatchTime() == 90.0).onTrue(DriveControls.driver.BeginRumble().alongWith(DriveControls.operator.BeginRumble()));
    new Trigger(() -> intake.isIntaked()).onTrue(DriveControls.driver.BeginRumble());
    
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
          });
    }
    return autoChooser.get();
  }

  public Command zeroPosition() {
    return pivot.PIDCommand(PivotArmConstants.PIVOT_ARM_MIN_ANGLE)
        .deadlineWith(
          intake.stop()
          .alongWith(shooter.stop())
          .alongWith(groundIntake.stop())
        );
  }

  public Command zeroShooter() {
    return (intake.stop())
        .alongWith(shooter.stop())
        .alongWith(groundIntake.stop());
  }

  public Command setAmpShooterSpeed() {
    return new FunctionalCommand(
        () -> {
          shooter.setRPM(ShooterConstants.defaultShooterSpeedRPM); // placeholder
                                                                                                            // speed
        },
        () -> {
          shooter.setRPM(ShooterConstants.defaultShooterSpeedRPM); // placeholder
                                                                                                            // speed
        },
        (interrupted) -> {
          if (!interrupted)
            return;

          shooter.stop();
        },
        () -> {
          return shooter.atSetpoint();
        },
        shooter);
  }

  public Command shootAmp() {
    return drive.pathfindToTrajectory(PathPlannerPath.fromPathFile("amp score"));
  }

  public Command shootAnywhere() {
    // implement this later using swerve to turn to desired target
    // move pivot arm
    // and calculate the speed required to shoot
    /*
     * if (DriveCommands.pointedAtSpeaker(drive)){
     * return rotateArm().andThen(shoot());
     * } else {
     * return
     * DriveCommands.turnSpeakerAngle(drive).alongWith(rotateArm()).andThen(shoot())
     * ;
     */

    // return DriveCommands.turnSpeakerAngle(drive).onlyIf(() ->
    // !DriveCommands.pointedAtSpeaker(drive)).alongWith(rotateArm()).andThen(shoot());
    return (rotateArm().andThen(shootNote())) // problem is here, both of these commands can't be robotContainer
        .deadlineWith(DriveCommands.joystickSpeakerPoint(
            drive,
            DriveControls.DRIVE_FORWARD,
            DriveControls.DRIVE_STRAFE)) //.andThen(zeroShooter())
            ;
      //the rotate arm method just keeps going, I don't know what's wrong with it
      //Maybe it's the shooter setRPM?

  }

  public Command shootSpeaker() {
    return (rotateArm().andThen(shootNote()));
  }

 
  public Command rotateArmtoSpeaker() {
    return new FunctionalCommand(
          () -> {},
          () -> {
            Logger.recordOutput("PivotArmSpeakerAngle", getAngle());
            pivot.setPID(getAngle());
            pivot.runPID();
          },
        (interrupted) -> {
          if (!interrupted) return;
          pivot.stop();
        },
        () -> {
          return pivot.atSetpoint();
        },
        pivot
    );
  }

  public Command lockOnSpeakerFull(){
    return (rotateArmtoSpeaker()) //problem is here, both of these commands can't be robotContainer
      .alongWith(DriveCommands.joystickSpeakerPoint(
        drive,
        DriveControls.DRIVE_FORWARD,
        DriveControls.DRIVE_STRAFE
      )
    );   
  }

  public Command rotateArm(){
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          Logger.recordOutput("PivotArmSpeakerAngle", getAngle());
          pivot.setPID(getAngle());
          pivot.runPID();
          //shooter.setRPM(getRPM());
        },
        (interrupted) -> {
          //shooter.stop();
          pivot.stop();
          //this should be stopping but it isn't
          //pivot.stop should be void, not return a command
        },
        () -> {

          if (pivot.atSetpoint()){
            Logger.recordOutput("atSetpoint", pivot.atSetpoint());
          }
          return pivot.atSetpoint();// && shooter.atSetpoint();
          //shooter can never get to that setpoint with a comically high speed probably
          //we pierce the heavens but bounce against the earth
          
        },
        pivot);
  }

  public Command shoot() {
    Logger.recordOutput("DistanceAway", getEstimatedDistance());
    if (pivot.atSetpoint() == true) {
      return intake.EjectLoopCommand(2).deadlineWith(shooter.runSpeed(() -> getRPM())
          .alongWith(pivot.PIDCommand(() -> getAngle())).alongWith(NoteVisualizer.shoot(drive)));
    } else {
      return null;
    }
  }

  public Command shootNote() {
    return

          //figure out why the shooter is so weaksauce
          //it's only shooting it out fast when I mash the button
          //probably has to do with the getRPM method
          shooter.runSpeed(() -> getRPM())
            .andThen(intake.EjectLoopCommand(8));
                //.alongWith(NoteVisualizer.shoot(drive)));//);
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
    //return 100000;
    //with a comically high speed it keeps running the point arm command but can't run the shooter command
    //will have to look into this later
  }

  // Gets angle based on distance from speaker, taking into account the actual
  // shooting position
  private double getAngle() {
    double armLength = PivotArmConstants.PivotArmSimConstants.kArmLength;
    double speakerHeight = Units.inchesToMeters(100.324);
    //double speakerHeight = 80.324;
    Transform2d targetTransform = drive.getPose().minus(FieldConstants.SpeakerPosition);
    double targetDistance = targetTransform.getTranslation().getNorm();
    double angle = Math.PI - (Math.acos(armLength/Math.sqrt(Math.pow(targetDistance, 2) + Math.pow(speakerHeight, 2))) + Math.atan(speakerHeight/targetDistance));
    Logger.recordOutput("calculatedangle", angle);
    return (angle);
    //comically high radian
    //return Lookup.getAngle(getEstimatedDistance());
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
    setPivotPose3d();
  }
  public boolean alwaysRun(){
    return true;
  }
}