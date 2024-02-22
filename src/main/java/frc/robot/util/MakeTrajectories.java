package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.Constants.DriveConstants.kPathConstraints;

public class MakeTrajectories {
    public static Command makeCustomAutoCommand(Command shoot, Drive drive) {
        Command auto = new InstantCommand();

        if (AutoChooser.shootOnStart.get()) {
            auto = auto.andThen(shoot);
        }

        auto = auto.andThen(drive.goPose(AutoChooser.NoteOneChooser.getSelected()));
        auto = auto.andThen(drive.goPose(AutoChooser.NoteOneShotChooser.getSelected()));
        auto = auto.andThen(shoot);

        // continue this stuff

        return auto;
    }
}
