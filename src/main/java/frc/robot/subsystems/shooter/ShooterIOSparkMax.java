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

  private SparkPIDController leftController; //velocity pid controller (left)
  private SparkPIDController rightController; //velocity pid controller (right)
  private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ShooterIOSparkMax() {
    leftMotor = new CANSparkFlex(SHOOTER_LEFT_ID, CANSparkFlex.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(SHOOTER_RIGHT_ID, CANSparkFlex.MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

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

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();
    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterPositionRotations = leftEncoder.getPosition();
    inputs.leftFlywheelVelocityRPM = leftEncoder.getVelocity();
    inputs.leftFlywheelAppliedVolts = leftMotor.getAppliedOutput() * 12;
    inputs.leftFlywheelOutputCurrent = leftMotor.getOutputCurrent();

    inputs.rightFlywheelPositionRotations = rightEncoder.getPosition();
    inputs.rightFlywheelVelocityRPM = rightEncoder.getVelocity();
    inputs.rightFlywheelAppliedVolts = rightMotor.getAppliedOutput() * 12;
    inputs.rightFlywheelOutputCurrent = rightMotor.getOutputCurrent();
  }
  
  @Override
  public void setLeftRPM(double rpm) {
    leftController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity,
        0,
        leftFF.calculate(rpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setRightRPM(double rpm) {
    rightController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity,
        0,
        rightFF.calculate(rpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setLeftVoltage(double volts){
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts){
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setLeftCharacterizationVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRightCharacterizationVoltage(double volts) {
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setLeftBrakeMode(boolean enabled) {
    leftMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public void setRightBrakeMode(boolean enabled) {
    rightMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public void setLeftPID(double p, double i, double d) {
    leftController.setP(p);
    leftController.setI(i);
    leftController.setD(d);
  }

  @Override
  public void setLeftFF(double kS, double kV, double kA) {
    leftFF = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setRightPID(double p, double i, double d) {
    rightController.setP(p);
    rightController.setI(i);
    rightController.setD(d);
  }

  @Override
  public void setRightFF(double s, double v, double a) {
    rightFF = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void stop() {
    setRPM(0.0, 0.0);
  }
}
