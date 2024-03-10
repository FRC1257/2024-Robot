package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoChooser {
    // Get the start position
    public static NoteChooser startChooser = new NoteChooser("Start");

    // Chose the note positions
    public static NoteChooser NoteOneChooser = new NoteChooser("NoteOne");
    public static NoteChooser NoteTwoChooser = new NoteChooser("NoteTwo");
    public static NoteChooser NoteThreeChooser = new NoteChooser("NoteThree");
    public static NoteChooser NoteFourChooser = new NoteChooser("NoteFour");
    // Chose the score positions
    public static NoteChooser NoteOneShotChooser = new NoteChooser("NoteOneShot");
    public static NoteChooser NoteTwoShotChooser = new NoteChooser("NoteTwoShot");
    public static NoteChooser NoteThreeShotChooser = new NoteChooser("NoteThreeShot");
    public static NoteChooser NoteFourShotChooser = new NoteChooser("NoteFourShot");
    
    // Get the start position
    public static void setupChoosers() {
        SmartDashboard.putBoolean("Auto/ShootOnStart", true);
        startChooser.setStartPosition();

        NoteOneChooser.setNotePosition();
        NoteTwoChooser.setNotePosition();
        NoteThreeChooser.setNotePosition();
        NoteFourChooser.setNotePosition();

        NoteOneShotChooser.setScorePosition();
        NoteTwoShotChooser.setScorePosition();
        NoteThreeShotChooser.setScorePosition();
        NoteFourShotChooser.setScorePosition();
    }

    public static Pose2d getStartPose() {
        return startChooser.getSelected();
    }
}
