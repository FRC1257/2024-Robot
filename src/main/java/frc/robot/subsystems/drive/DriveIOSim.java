package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Drivetrain.*;

public class DriveIOSim implements DriveIO {
  /* double batteryMoi = 12.5 / 2.2 * Math.pow(Units.inchesToMeters(10), 2);
  double gearboxMoi =
      (2.8 * 2 / 2.2 + 2.0 ) * Math.pow(Units.inchesToMeters(26.0 / 2.0), 2); */
  // private DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(KitbotMotor.kDoubleNEOPerSide.value, KitbotGearing.k10p71.value, Units.lbsToKilograms(60), batteryMoi + gearboxMoi, DRIVE_WHEEL_DIAM_M, DRIVE_TRACK_WIDTH_M, null);
  private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private PIDController leftController = new PIDController(5, 0, 0);
  private PIDController rightController = new PIDController(5, 0, 0);

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    sim.update(0.02);
    inputs.leftPositionRad = sim.getLeftPositionMeters() / Drive.WHEEL_RADIUS_METERS;
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Drive.WHEEL_RADIUS_METERS;
    inputs.rightPositionRad = sim.getRightPositionMeters() / Drive.WHEEL_RADIUS_METERS;
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / Drive.WHEEL_RADIUS_METERS;
    inputs.gyroYawRad = sim.getHeading().getRadians() * -1;
    inputs.timestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    sim.setInputs(MathUtil.clamp(leftVolts, -12.0, 12.0), MathUtil.clamp(rightVolts, -12.0, 12.0));
  }

  @Override
  public double getLeftPositionMeters() {
    return sim.getLeftPositionMeters();
  }

  @Override
  public double getRightPositionMeters() {
    return sim.getRightPositionMeters();
  }

  @Override
  public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
    leftController.setSetpoint(wheelSpeeds.leftMetersPerSecond);
    rightController.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    setVoltage(leftController.calculate(sim.getLeftVelocityMetersPerSecond()), rightController.calculate(sim.getRightVelocityMetersPerSecond()));
  }

  @Override
  public void zero() {
    
  }

  @Override
  public double getTrackWidth() {
    return Units.inchesToMeters(26);
  }

  @Override
  public double getRobotAngle() {
    return sim.getHeading().getRadians() * -1;
  }
}
