package frc.robot.util.autonomous;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

public class NoteChooser {

    private LoggedDashboardChooser<Pose2d> position;

    public NoteChooser(String name){
        position = new LoggedDashboardChooser<Pose2d>(name);
    }

    public void setNotePosition(){
        Pose2d[] NOTE_POSITIONS = FieldConstants.NOTE_POSITIONS();
        position.addDefaultOption("Note 1", NOTE_POSITIONS[0]);
        for (int i = 1; i < NOTE_POSITIONS.length; i++){
            position.addOption("Note " + (i + 1), NOTE_POSITIONS[i]);
        }
    }

    public void setStartPosition(){
        Pose2d[] START_POSITIONS = FieldConstants.START_POSITIONS();
        position.addDefaultOption("Top", START_POSITIONS[0]);
        position.addOption("Center", START_POSITIONS[1]);
        position.addOption("Bottom", START_POSITIONS[2]);
        position.addOption("Really Bottom", START_POSITIONS[3]);
        position.addOption("Really Really Bottom", START_POSITIONS[3]);
    }

    public void setScorePosition(){
        Pose2d[] SCORE_POSITIONS = FieldConstants.SCORE_POSITIONS();
        position.addDefaultOption("Top", SCORE_POSITIONS[0]);
        position.addOption("Center", SCORE_POSITIONS[1]);
        position.addOption("Bottom", SCORE_POSITIONS[2]);
        position.addOption("Near Line Top", SCORE_POSITIONS[3]);
        position.addOption("Near Line Bottom", SCORE_POSITIONS[3]);
    }

    public Pose2d getSelected(){
        Pose2d selected_position = position.get();

        if (selected_position == null){
            System.out.println("Null value for some reason");
            selected_position = FieldConstants.NOTE_POSITIONS()[0];
        }

        return selected_position;
    }

    
}