package frc.robot.subsystems.trapClaw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapClaw extends SubsystemBase {
    // IO interface to communicate with motor
    private TrapClawIO io;
    private TrapClawIOInputsAutoLogged inputs;

    public TrapClaw(TrapClawIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this); // Log claw data on dashboard
    }

    // Runs periodically and updates, processes, and logs inputs
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    // Moves motor a set speed (-1 <= speed <= 1)
    public void move(double speed) {
        io.setVoltage(speed * 12);
    }

    // Command to turn wheel a set speed (-1 <= speed <= 1)
    public Command TurnWheelCommand(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> move(speed),
            (stop) -> move(0),
            () -> false,
            this
        );
    }
}
