package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.Constants.DriveConstants.*;

public class TurnAngleCommand extends Command {
    private Drive drive;
    private Rotation2d angle;

    private static PIDController angleController = new PIDController(kTurnSpeakerP, kTurnSpeakerI, kTurnSpeakerD);
    
    public TurnAngleCommand(Drive drive, Rotation2d angle) {
        addRequirements(drive);
        this.drive = drive;
        this.angle = angle;

        angleController.setTolerance(kTurnSpeakerTolerance, kTurnSpeakerRateTolerance);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        angleController.reset();
        angleController.setSetpoint(angle.getRadians());
    }

    @Override
    public void execute() {
        // No movement
        double linearMagnitude = 0;
        Rotation2d linearDirection = new Rotation2d();
        
        double omega = angleController.calculate(drive.getRotation().getRadians(), angle.getRadians());

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
            new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();

        // Convert to robot relative speeds & send command
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec(),
                drive.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
