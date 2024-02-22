package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChooser {
    public static LoggedDashboardBoolean shootOnStart = new LoggedDashboardBoolean("ShootOnStart", true);

    // Get the start position
    public static NoteChooser startChooser = new NoteChooser("Start");

    // Chose the note positions
    public static NoteChooser NoteOneChooser = new NoteChooser("NoteOne");
    public static NoteChooser NoteTwoChooser = new NoteChooser("NoteTwo");
    
    // Chose the score positions
    public static NoteChooser NoteOneShotChooser = new NoteChooser("NoteOneShot");
    public static NoteChooser NoteTwoShotChooser = new NoteChooser("NoteTwoShot");

    // Get the start position
    public static void setupChoosers() {
        startChooser.setStartPosition();

        NoteOneChooser.setNotePosition();
        NoteTwoChooser.setNotePosition();

        NoteOneShotChooser.setScorePosition();
        NoteTwoShotChooser.setScorePosition();
    }

}
