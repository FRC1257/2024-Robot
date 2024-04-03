package frc.robot.util.autonomous;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class MakeAutos {
    /* public static Command driveAndIntakeNote(int noteOrder, Drive drive, Supplier<Command> zeroPivot, Supplier<Command> intakeRun, Supplier<Command> intakeWhile) {
        Pose2d notePose = null;
        switch(noteOrder) {
            case 1:
                notePose = AutoChooser.NoteOneChooser.getSelected();
                break;
            case 2:
                notePose = AutoChooser.NoteTwoChooser.getSelected();
                break;
            case 3:
                notePose = AutoChooser.NoteThreeChooser.getSelected();
                break;
            case 4:
                notePose = AutoChooser.NoteFourChooser.getSelected();
                break;
        }
        return ((drive.goToPose(notePose))
                    .deadlineWith(intakeRun.get()))
                    .andThen(intakeWhile.get());
    } */

    public static Command makeAutoCommand(Drive drive, Supplier<Command> shoot, Supplier<Command> shootSpeaker, Supplier<Command> intakeWhile, Supplier<Command> zeroPivot, BooleanSupplier hasNote) {
        return new SequentialCommandGroup(
            shootSpeaker.get(),
            // First
            drive.goToPose(AutoChooser.NoteOneChooser.getSelected()).alongWith(intakeWhile.get()).deadlineWith(zeroPivot.get()), 
            drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()).andThen(shoot.get()).onlyIf(hasNote),
            // Second
            drive.goToPose(AutoChooser.NoteTwoChooser.getSelected()).alongWith(intakeWhile.get()).deadlineWith(zeroPivot.get()),
            drive.goToPose(AutoChooser.NoteTwoShotChooser.getSelected()).andThen(shoot.get()).onlyIf(hasNote),
            // Third
            drive.goToPose(AutoChooser.NoteThreeChooser.getSelected()).alongWith(intakeWhile.get()).deadlineWith(zeroPivot.get()),
            drive.goToPose(AutoChooser.NoteThreeShotChooser.getSelected()).andThen(shoot.get()).onlyIf(hasNote),
            // Fourth
            drive.goToPose(AutoChooser.NoteFourChooser.getSelected()).alongWith(intakeWhile.get()).deadlineWith(zeroPivot.get()),
            drive.goToPose(AutoChooser.NoteFourShotChooser.getSelected()).andThen(shoot.get()).onlyIf(hasNote),
            zeroPivot.get()
        );
    }

    /* public static Command makeAutoTrajectoryCommand(Drive drive, Supplier<Command> shoot, Supplier<Command> intakeCommand, Supplier<Command> intakeWhile, Supplier<Command> zeroPivot, Command shooterPrep) {
        return new SequentialCommandGroup.andThen(shoot.get())(
    
            new WaitCommand(0.2.andThen(shoot.get())).andThen(shoot.get()),
            zeroPivot.get(),
            drive.driveFromPoseToPose(AutoChooser.startChooser.getSelected(), AutoChooser.NoteOneChooser.getSelected()).deadlineWith(intakeCommand.get()), //gotopose is the issue
            drive.driveFromPoseToPose(AutoChooser.NoteOneChooser.getSelected(),AutoChooser.NoteOneShotChooser.getSelected()).andThen(shoot.get()),
            zeroPivot.get(),
            drive.driveFromPoseToPose(AutoChooser.NoteOneShotChooser.getSelected(), AutoChooser.NoteTwoChooser.getSelected()).deadlineWith(intakeCommand.get()),
            drive.driveFromPoseToPose(AutoChooser.NoteTwoChooser.getSelected(),AutoChooser.NoteTwoShotChooser.getSelected()).andThen(shoot.get()),
            zeroPivot.get(),
            drive.driveFromPoseToPose(AutoChooser.NoteTwoShotChooser.getSelected(), AutoChooser.NoteThreeChooser.getSelected()).deadlineWith(intakeCommand.get()),
            drive.driveFromPoseToPose(AutoChooser.NoteThreeChooser.getSelected(), AutoChooser.NoteThreeShotChooser.getSelected()).andThen(shoot.get()),
            zeroPivot.get(),
            drive.driveFromPoseToPose(AutoChooser.NoteThreeShotChooser.getSelected(),AutoChooser.NoteFourChooser.getSelected()).deadlineWith(intakeCommand.get()),
            drive.driveFromPoseToPose(AutoChooser.NoteFourChooser.getSelected(), AutoChooser.NoteFourShotChooser.getSelected()).andThen(shoot.get()),
            zeroPivot.get()
        );
    } */
    
}
