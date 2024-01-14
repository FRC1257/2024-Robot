package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class DriveIOCIM implements DriveIO {
  private final CANSparkMax leftLeader;
  private final CANSparkMax rightLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  private final GyroIOReal gyro;

  public DriveIOCIM() {
    leftLeader = new CANSparkMax(17, MotorType.kBrushed);
    rightLeader = new CANSparkMax(4, MotorType.kBrushed); // coast
    leftFollower = new CANSparkMax(11, MotorType.kBrushed);
    rightFollower = new CANSparkMax(10, MotorType.kBrushed); // coast

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kCoast);

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setInverted(true);
    rightLeader.setInverted(false);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    gyro = GyroIOReal.getInstance();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = 0;
    inputs.rightPositionRad = 0;
    inputs.leftVelocityRadPerSec = leftLeader.getAppliedOutput();
    inputs.rightVelocityRadPerSec = rightLeader.getAppliedOutput();
    inputs.gyroYawRad = gyro.getYawAngle();
    inputs.gyroRollPitchYawRad[0] = gyro.getRollAngle();
    inputs.gyroRollPitchYawRad[1] = gyro.getPitchAngle();
    inputs.gyroRollPitchYawRad[2] = gyro.getYawAngle();
    inputs.timestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  /* No encoders so no position or velocity control works */
  @Override
  public double getLeftPositionMeters() {
    return 0;
  }

  @Override
  public double getRightPositionMeters() {
    return 0;
  }

  @Override
  public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
  }

  @Override
  public void zero() {
    GyroIOReal.getInstance().zeroAll();
  }

  @Override
  public double getRobotAngle() {
    return GyroIOReal.getInstance().getYawAngle();
  }
  
}
