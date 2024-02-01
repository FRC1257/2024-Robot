//this is the commands for the shooter
package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    private shooterMotor shooter;
    private leftSpeed lSpeed;
    private rightSpeed rSpeed;
    private SubsystemBase subsystem;

    public ShooterSpeedCommand(shooterMotor shooter, leftSpeed lSpeed, rightSpeed rSpeed) {
    this.shooter = shooter;
    this.rspeed = rSpeed
    this.lspeed = lSpeed
    
    addRequirements(shooter);
    }
//make sure that the right speed negatives and positives are correct
public Command intake() {
    return new FunctionalCommand(
    () -> io.setrSpeed(-12.0),
    () -> io.setlSpeed(12.0),
    (interrupted) -> io.setSpeed(speed:0.0),
    this;
    );
}

public Command release() {
    return new FunctionalCommand(
    () -> io.setrSpeed(100.0),
    () -> io.setlSpeed(-100.0),
    (interrupted) -> io.setSpeed(speed:0.0),
    this;
    );
}
}