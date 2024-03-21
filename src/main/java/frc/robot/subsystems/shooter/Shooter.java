package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.*;
import frc.robot.util.misc.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber leftkP =
      new LoggedTunableNumber("Shooter/leftkP", leftShooter.kP());
  private static final LoggedTunableNumber leftkI =
      new LoggedTunableNumber("Shooter/leftkI", leftShooter.kI());
  private static final LoggedTunableNumber leftkD =
      new LoggedTunableNumber("Shooter/leftkD", leftShooter.kD());
  private static final LoggedTunableNumber leftkS =
      new LoggedTunableNumber("Shooter/leftkS", leftShooter.kS());
  private static final LoggedTunableNumber leftkV =
      new LoggedTunableNumber("Shooter/leftkV", leftShooter.kV());
  private static final LoggedTunableNumber leftkA =
      new LoggedTunableNumber("Shooter/leftkA", leftShooter.kA());
  private static final LoggedTunableNumber rightkP =
      new LoggedTunableNumber("Shooter/rightkP", rightShooter.kP());
  private static final LoggedTunableNumber rightkI =
      new LoggedTunableNumber("Shooter/rightkI", rightShooter.kI());
  private static final LoggedTunableNumber rightkD =
      new LoggedTunableNumber("Shooter/rightkD", rightShooter.kD());
  private static final LoggedTunableNumber rightkS =
      new LoggedTunableNumber("Shooter/rightkS", rightShooter.kS());
  private static final LoggedTunableNumber rightkV =
      new LoggedTunableNumber("Shooter/rightkV", rightShooter.kV());
  private static final LoggedTunableNumber rightkA =
      new LoggedTunableNumber("Shooter/rightkA", rightShooter.kA());
  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", shooterToleranceRPM);

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private boolean characterizing = false;
 
  private double leftSetpointRPM, rightSetpointRPM = 0;
  private double leftMotorVoltage, rightMotorVoltage = 0;

  public Shooter(ShooterIO io) {
    shooterIO = io;
    shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get());
    shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get());
    shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get());
    shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get());
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get()),
        leftkP,
        leftkI,
        leftkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get()),
        leftkS,
        leftkV,
        leftkA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get()),
        rightkP,
        rightkI,
        rightkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get()),
        rightkS,
        rightkV,
        rightkA);

    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (DriverStation.isDisabled()) {
      shooterIO.stop();
    } else {
      if (!characterizing) {
        // shooterIO.setRPM(leftSpeedRpm.get(), rightSpeedRpm.get());
      }
    }

    Logger.recordOutput("Shooter/LeftRPM", shooterInputs.leftFlywheelVelocityRPM);
    Logger.recordOutput("Shooter/RightRPM", shooterInputs.rightFlywheelVelocityRPM);
  }

  public void runLeftVolts(double volts) {
    shooterIO.setLeftVoltage(volts);
  }

  public void runRightVolts(double volts) {
    shooterIO.setRightVoltage(volts);
  }

  public double getLeftCharacterizationVelocity() {
    return shooterInputs.leftFlywheelVelocityRPM;
  }

  public void setCharacterizing(boolean characterizing) {
    this.characterizing = characterizing;
  }

  public double getRightCharacterizationVelocity() {
    return shooterInputs.rightFlywheelVelocityRPM;
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(shooterInputs.leftFlywheelVelocityRPM - leftSetpointRPM)
            <= shooterTolerance.get()
        && Math.abs(shooterInputs.rightFlywheelVelocityRPM - rightSetpointRPM)
            <= shooterTolerance.get();
  }

  public Command runVoltage(DoubleSupplier volts) {
    return new FunctionalCommand(
      //() -> setRPM(speed.getAsDouble(), speed.getAsDouble()),
      () -> setVoltage(volts, volts), //no PID for now
      () -> {setVoltage(volts, volts);},
      (interrupted) -> {
        if (interrupted) {
          shooterIO.stop();
        }
      },
      () -> false,
      this
    );
  }

  public Command runVoltage(double volts) {
    return runVoltage(() -> volts);
  }

  public void setVoltage(DoubleSupplier leftVoltage, DoubleSupplier rightVoltage){
    leftMotorVoltage = leftVoltage.getAsDouble() * 10;
    rightMotorVoltage = rightVoltage.getAsDouble() * 10;
    shooterIO.setVoltage(leftMotorVoltage, rightMotorVoltage);
  }

  public void setVoltage(double volts){
    shooterIO.setVoltage(volts, volts);
  }

 
  public void setRPM(double leftRPM, double rightRPM) {
    leftSetpointRPM = leftRPM; //RPM setpoint is being set here
    rightSetpointRPM = rightRPM;
    shooterIO.setRPM(leftRPM, rightRPM);
  }

  public void setRPM(double rpm) {
    setRPM(rpm, rpm);
  }

  public void setRPM(DoubleSupplier leftRPM, DoubleSupplier rightRPM) {
    leftSetpointRPM = leftRPM.getAsDouble();
    rightSetpointRPM = rightRPM.getAsDouble();
    shooterIO.setRPM(leftSetpointRPM, rightSetpointRPM);
  }

  public Command stop() {
    return runVoltage(() -> 0);
  }

  public double getLeftSpeedMetersPerSecond() {
    return shooterInputs.leftFlywheelVelocityRPM * wheelRadiusM * 2 * Math.PI / 60;
  }

  public double getRightSpeedMetersPerSecond() {
    return shooterInputs.rightFlywheelVelocityRPM * wheelRadiusM * 2 * Math.PI / 60;
  }
}