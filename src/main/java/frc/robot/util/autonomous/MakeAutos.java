package frc.robot.util.autonomous;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class MakeAutos {
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
}
