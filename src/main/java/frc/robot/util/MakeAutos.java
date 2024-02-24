package frc.robot.util;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class MakeAutos {
    public static Command makeAutoCommand(Drive drive, Command shoot, Command intake, Command intakeWhile) {
        return new SequentialCommandGroup(
            shoot,
            drive.goToPose(AutoChooser.NoteOneChooser.getSelected()),
            drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()),
            shoot,
            drive.goToPose(AutoChooser.NoteTwoChooser.getSelected()),
            drive.goToPose(AutoChooser.NoteTwoShotChooser.getSelected()),
            shoot,
            drive.goToPose(AutoChooser.NoteThreeChooser.getSelected()),
            drive.goToPose(AutoChooser.NoteThreeShotChooser.getSelected()),
            shoot,
            drive.goToPose(AutoChooser.NoteFourChooser.getSelected()),
            drive.goToPose(AutoChooser.NoteFourShotChooser.getSelected()),
            shoot
        );
    }
    
}
