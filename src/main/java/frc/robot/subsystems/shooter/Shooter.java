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
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", shooter.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", shooter.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", shooter.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/kS", shooter.kS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/kV", shooter.kV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/kA", shooter.kA());
  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", shooterToleranceRPM);

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private boolean characterizing = false;
 
  private double setpoint = 0;
  private double voltage = 0;

  public Shooter(ShooterIO io) {
    shooterIO = io;
    shooterIO.setPID(kP.get(), kI.get(), kD.get());
    shooterIO.setFF(kS.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setPID(kP.get(), kI.get(), kD.get()),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setFF(kS.get(), kV.get(), kA.get()),
        kS,
        kV,
        kA);

    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (DriverStation.isDisabled()) {
      shooterIO.stop();
    } else {
      if (!characterizing) {
        // shooterIO.setRPM(SpeedRpm.get(), rightSpeedRpm.get());
      }
    }

    Logger.recordOutput("Shooter/setpoint", setpoint);
  }

  public void runCharacterizationVolts(double volts) {
    shooterIO.setCharacterizationVoltage(volts);
  }

  public double getCharacterizationVelocity() {
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
    return Math.abs(shooterInputs.leftFlywheelVelocityRPM - setpoint)
            <= shooterTolerance.get();
  }

  public Command runSpeed(double speed) {
    return new RunCommand(
      () -> setRPM(speed),
      this
    );
  }

  public Command runSpeed(DoubleSupplier speed) {
    return new FunctionalCommand(
      () -> setRPM(speed.getAsDouble()),
      () -> {},
      (interrupted) -> {
        if (interrupted) {
          shooterIO.stop();
        }
      },
      () -> false,
      this
    );
  }

  public Command runVoltage(DoubleSupplier volts) {
    return new FunctionalCommand(
      //() -> setRPM(speed.getAsDouble(), speed.getAsDouble()),
      () -> setVoltage(volts), //no PID for now
      () -> {setVoltage(volts);},
      (interrupted) -> {
        if (interrupted) {
          shooterIO.stop();
        }
      },
      () -> false,
      this
    );
  }

  public Command runCommmand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
      //() -> setRPM(speed.getAsDouble(), speed.getAsDouble()),
      () -> run(speedSupplier), //no PID for now
      () -> {run(speedSupplier);},
      (interrupted) -> {
        if (interrupted) {
          shooterIO.stop();
        }
      },
      () -> false,
      this
    );
  }

  public void setVoltage(DoubleSupplier voltageSupplier){
    voltage = voltageSupplier.getAsDouble() * 12;
    
    shooterIO.setVoltage(voltage);
  }

  public void run(DoubleSupplier speed){
    Logger.recordOutput("ShooterSpeed", speed.getAsDouble()*12);
    shooterIO.run(speed.getAsDouble()*12);
  }

  public void setRPM(double RPM) {
    setpoint = RPM;
    shooterIO.setRPM(setpoint);
  }

  public void setRPM(DoubleSupplier RPM) {
    setpoint = RPM.getAsDouble();
    shooterIO.setRPM(setpoint);
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
