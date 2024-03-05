package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ElectricalLayout.SHOOTER_LEFT_ID;
import static frc.robot.Constants.ElectricalLayout.SHOOTER_RIGHT_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.flywheelReduction;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterIOSparkMax implements ShooterIO {
  private CANSparkFlex leftMotor;
  private CANSparkFlex rightMotor;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkPIDController pidController; //velocity pid controller (left)
  private SparkPIDController rightController; //velocity pid controller (right)
  private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ShooterIOSparkMax() {
    leftMotor = new CANSparkFlex(SHOOTER_LEFT_ID, CANSparkFlex.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(SHOOTER_RIGHT_ID, CANSparkFlex.MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    //leftMotor.enableVoltageCompensation(12.0);
    //rightMotor.enableVoltageCompensation(12.0);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    // rotations, rps
    leftEncoder.setPositionConversionFactor(1.0 / flywheelReduction);
    rightEncoder.setPositionConversionFactor(1.0 / flywheelReduction);
    leftEncoder.setVelocityConversionFactor(1.0 / flywheelReduction);
    rightEncoder.setVelocityConversionFactor(1.0 / flywheelReduction);

    pidController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();
    pidController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterPositionRotations = leftEncoder.getPosition();
    inputs.leftFlywheelVelocityRPM = leftEncoder.getVelocity();
    inputs.leftFlywheelAppliedVolts = leftMotor.getAppliedOutput();
    inputs.leftFlywheelOutputCurrent = leftMotor.getOutputCurrent();

    inputs.rightFlywheelPositionRotations = rightEncoder.getPosition();
    inputs.rightFlywheelVelocityRPM = rightEncoder.getVelocity();
    inputs.rightFlywheelAppliedVolts = rightMotor.getAppliedOutput();
    inputs.rightFlywheelOutputCurrent = rightMotor.getOutputCurrent();
  }

  @Override
  public void setRPM(double rpm) {
    pidController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity,
        0,
        leftFF.calculate(rpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts){
    Logger.recordOutput("thing", true);
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  } //you didn't change the default voltage little bro

  @Override
  public void run(double speed) {
    Logger.recordOutput("trytry", MathUtil.clamp(speed, -1, 1));
    leftMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  @Override
  public void setCharacterizationVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setPID(double p, double i, double d) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    leftFF = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
