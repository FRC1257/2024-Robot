// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.Constants.PivotArm.PivotArmSimConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCIM;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalon;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;


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
  private final PivotArm pivot;
  private Mechanism2d mech = new Mechanism2d(3, 3);
  
  // Controllers
  private final CommandSnailController driver = new CommandSnailController(0);
  private final CommandSnailController operator = new CommandSnailController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean isBlue = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(new DriveIOTalon(), new VisionIOPhoton(), new Pose2d());
        pivot = new PivotArm(new PivotArmIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(new DriveIOSim(), new VisionIOSim(), new Pose2d());
        pivot = new PivotArm(new PivotArmIOSim());
        break;
      case TEST:
        drive = new Drive(new DriveIOCIM(), new VisionIOPhoton(), new Pose2d());
        pivot = new PivotArm(new PivotArmIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(new DriveIO() {}, new VisionIO() {}, new Pose2d());
        pivot = new PivotArm(new PivotArmIO() {});
        break;
    }

    // Set up robot state manager

    MechanismRoot2d root = mech.getRoot("elevator", 1, 0.5);
    // add subsystem mechanisms
    SmartDashboard.putData("Arm Mechanism", mech);
    
    setMechanism(getArmMechanism(), pivot);
    isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Spin", new SpinAuto(drive));
    
    // Configure the button bindings
    configureButtonBindings();
  }
   public void setMechanism(MechanismLigament2d mechanism, PivotArm pivot) {
        this.pivot.setMechanism(mechanism);
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism, PivotArm pivot) {
        return this.pivot.getArmMechanism().append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", 2, 0, 5, new Color8Bit(Color.kAqua));
    }
   
    
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    drive.setDefaultCommand(
        new RunCommand(() -> drive.driveArcade(driver.getDriveForward(), driver.getDriveTurn()), drive));

    driver.rightBumper().onTrue(
      new StartEndCommand(() -> drive.startSlowMode(), () -> drive.stopSlowMode(), drive)
    );

    // cancel trajectory
    driver.getY().onTrue(drive.endTrajectoryCommand());
    pivot.setDefaultCommand(
        new RunCommand(() -> pivot.move(operator.getLeftY()), pivot));
    // these are triggers that run the subsystem's command
    operator.getB().onTrue(pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MAX_ANGLE));
    operator.getX().onTrue(pivot.PIDCommand(Constants.PivotArm.PIVOT_ARM_MIN_ANGLE));

   
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
