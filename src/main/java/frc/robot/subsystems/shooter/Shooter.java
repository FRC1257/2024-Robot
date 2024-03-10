package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

import static frc.robot.Constants.ElectricalLayout;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private double leftSetpointRPM, rightSetpointRPM = 0;
  private double leftMotorVoltage, rightMotorVoltage = 0;

    private CANSparkFlex leftMotor;
  private CANSparkFlex rightMotor;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  public Shooter() {
    leftMotor = new CANSparkFlex(ElectricalLayout.SHOOTER_LEFT_ID, CANSparkFlex.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(ElectricalLayout.SHOOTER_RIGHT_ID, CANSparkFlex.MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    setLeftBrakeMode(false);
    setRightBrakeMode(false);

    rightMotor.setInverted(true);

    leftMotor.setSmartCurrentLimit(Constants.NEO_VORTEX_CURRENT_LIMIT);
    rightMotor.setSmartCurrentLimit(Constants.NEO_VORTEX_CURRENT_LIMIT);
    //leftMotor.enableVoltageCompensation(12.0);
    //rightMotor.enableVoltageCompensation(12.0);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    // rotations, rps
    leftEncoder.setPositionConversionFactor(1.0);
    rightEncoder.setPositionConversionFactor(1.0);
    leftEncoder.setVelocityConversionFactor(1.0);
    rightEncoder.setVelocityConversionFactor(1.0);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  public void periodic() {
    //SmartDashboard.putNumber("Shooter/Voltage", getBusVoltage());
  }

  public Command runVoltage(DoubleSupplier volts) {
    return new FunctionalCommand(
      () -> {}, //no PID for now
      () -> setVoltage(volts, volts),
      (interrupted) -> stop(),
      () -> false,
      this
    );
  }

  public Command runVoltage(double volts) {
    return runVoltage(() -> volts);
  }

  public void setVoltageSupplier(DoubleSupplier leftVoltage, DoubleSupplier rightVoltage){
    leftMotorVoltage = leftVoltage.getAsDouble() * 10;
    rightMotorVoltage = rightVoltage.getAsDouble() * 10;
    //setVoltage(leftMotorVoltage, defaultShooterSpeedRPM); bruh I love autcorrect
    setVoltage(leftMotorVoltage, rightMotorVoltage);
  }

  public Command stopCommand() {
    return runVoltage(() -> 0);
  }

  public void setLeftVoltage(double volts){
    leftMotor.setVoltage(volts);
  }

  public void setRightVoltage(double volts){
    rightMotor.setVoltage(volts);
  }

  public void setLeftCharacterizationVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  public void setRightCharacterizationVoltage(double volts) {
    rightMotor.setVoltage(volts);
  }

  public void setLeftBrakeMode(boolean enabled) {
    leftMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  public void setRightBrakeMode(boolean enabled) {
    rightMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  public void stop() {
    setLeftVoltage(0.0);
    setRightVoltage(0);
  }

  public void setVoltage(double left, double right) {
    setLeftVoltage(left);
    setRightVoltage(right);
  }

  public void setVoltage(DoubleSupplier left, DoubleSupplier right) {
    setLeftVoltage(left.getAsDouble());
    setRightVoltage(right.getAsDouble());
  }
}
