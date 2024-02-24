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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class MakeAutos {
    public static Command makeCustomAutoCommand(Drive drive, Command shoot, Command intake, Command intakeWhile) {
        ArrayList<Command> commands = new ArrayList<Command>();

        if (AutoChooser.shootOnStart.get()) {
            commands.add(shoot);
        }

        // Add commadns to the list
        commands.add(drive.goToPose(AutoChooser.NoteOneChooser.getSelected()).alongWith(intake)); // add intake stuff
                                                                                                  // here too
        // commmands.add(intakeWhile); // use vision to detect the note
        commands.add(drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()));
        commands.add(shoot);

        // Add commadns to the list
        commands.add(drive.goToPose(AutoChooser.NoteOneChooser.getSelected()).alongWith(intake)); // add intake stuff
                                                                                                  // here too
        // commmands.add(intakeWhile); // use vision to detect the note
        commands.add(drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()));
        commands.add(shoot);

        // Add commadns to the list
        commands.add(drive.goToPose(AutoChooser.NoteTwoChooser.getSelected()).alongWith(intake)); // add intake stuff
                                                                                                  // here too
        // commmands.add(intakeWhile); // use vision to detect the note
        commands.add(drive.goToPose(AutoChooser.NoteTwoShotChooser.getSelected()));
        commands.add(shoot);

        // Add commadns to the list
        commands.add(drive.goToPose(AutoChooser.NoteThreeChooser.getSelected()).alongWith(intake)); // add intake stuff
                                                                                                    // here too
        // commmands.add(intakeWhile); // use vision to detect the note
        commands.add(drive.goToPose(AutoChooser.NoteThreeShotChooser.getSelected()));
        commands.add(shoot);

        // Add commadns to the list
        commands.add(drive.goToPose(AutoChooser.NoteFourChooser.getSelected()).alongWith(intake)); // add intake stuff
                                                                                                   // here too
        // commmands.add(intakeWhile); // use vision to detect the note
        commands.add(drive.goToPose(AutoChooser.NoteFourShotChooser.getSelected()));
        commands.add(shoot);

        // Convert the arraylist to a command
        SequentialCommandGroup auto = new SequentialCommandGroup(commands.toArray(Command[]::new));
        return auto;
    }
}
