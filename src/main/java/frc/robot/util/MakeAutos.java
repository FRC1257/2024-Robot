package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class MakeAutos {
    public static Command makeAutoCommand(Drive drive, Supplier<Command> shoot, Supplier<Command> intakeCommand, Supplier<Command> intakeWhile) {
        // shoot needs to be commented out to work
        // TODO fix this
        return new SequentialCommandGroup(
            shoot.get(),
            drive.goToPose(AutoChooser.NoteOneChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteOneShotChooser.getSelected()),
            shoot.get(),
            drive.goToPose(AutoChooser.NoteTwoChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteTwoShotChooser.getSelected()),
            shoot.get(),
            drive.goToPose(AutoChooser.NoteThreeChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteThreeShotChooser.getSelected()),
            shoot.get(),
            drive.goToPose(AutoChooser.NoteFourChooser.getSelected()).deadlineWith(intakeCommand.get()),
            intakeWhile.get(),
            drive.goToPose(AutoChooser.NoteFourShotChooser.getSelected()),
            shoot.get()
        );
    }
    
}
