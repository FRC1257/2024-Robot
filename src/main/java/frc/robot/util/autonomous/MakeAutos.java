package frc.robot.util.autonomous;

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

    public static Command makeAutoCommand(Drive drive, Supplier<Command> shoot, Supplier<Command> intakeCommand, Supplier<Command> intakeWhile, Supplier<Command> zeroPivot, Command shooterPrep) {
        return new SequentialCommandGroup(
            //shoot.get(),
            new WaitCommand(0.2).andThen(shoot.get()).deadlineWith(shooterPrep),
            zeroPivot.get(),
            drive.goToPose(AutoChooser.NoteOneChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()),
            shoot.get(),
            zeroPivot.get(),
            drive.goToPose(AutoChooser.NoteTwoChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteTwoShotChooser.getSelected()),
            shoot.get(),
            zeroPivot.get(),
            drive.goToPose(AutoChooser.NoteThreeChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteThreeShotChooser.getSelected()),
            shoot.get(),
            zeroPivot.get(),
            drive.goToPose(AutoChooser.NoteFourChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteFourShotChooser.getSelected()),
            shoot.get(),
            zeroPivot.get()
        );
    }
    
}
