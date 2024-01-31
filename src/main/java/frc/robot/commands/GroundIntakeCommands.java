package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GroundIntake;
import static frc.robot.Constants.GroundIntake.*;
import java.util.function.DoubleSupplier;

public class GroundIntakeCommands {
    private GroundIntakeCommands() {}

    public static Command groundIntakeCommand(
        GroundIntake groundIntake,
        DoubleSupplier voltageSupplier) {
            return Commands.run(
                () -> {
                    if (SmartDashboard.getBoolean("Ground Intake Break Beam", breakBeamSensor.get()) == true) {
                        groundIntake.setVoltage(0.0);
                    }
                    double voltage = voltageSupplier.getAsDouble();
                    groundIntake.setVoltage(voltage);
                },
                groundIntake::breakBeamSensor,
                () -> groundIntake.setBrake(true),
                groundIntake::setBrake(false)
            );
        }
}
