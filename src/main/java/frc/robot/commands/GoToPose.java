package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.autonomous.MakeAutos;

public class GoToPose extends Command {
    private Drive drive;
    private Pose2d pose;
    private PathPlannerPath path;

    public GoToPose(Drive drive, Pose2d pose) {
        this.drive = drive;
        this.pose = pose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.goToThaPose(pose).schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
