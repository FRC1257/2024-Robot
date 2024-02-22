package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.FieldConstants.*;

public class NoteChooser {

    private String name;
    private LoggedDashboardChooser<Pose2d> position;

    public NoteChooser(String name){
        this.name = name;
        position = new LoggedDashboardChooser<Pose2d>(name);
    }

    public void setNotePosition(){
        position.addDefaultOption("Note 1", NOTE_POSITIONS[0]);
        for (int i = 1; i < NOTE_POSITIONS.length; i++){
            position.addOption("Note " + (i + 1), NOTE_POSITIONS[i]);
        }
    }

    public void setStartPosition(){
        position.addDefaultOption("Top", START_POSITIONS[0]);
        position.addOption("Center", START_POSITIONS[1]);
        position.addOption("Bottom", START_POSITIONS[2]);
        position.addOption("Really Bottom", START_POSITIONS[3]);
        position.addOption("Really Really Bottom", START_POSITIONS[3]);
    }

    public void setScorePosition(){
        position.addDefaultOption("Top", SCORE_POSITIONS[0]);
        position.addOption("Center", SCORE_POSITIONS[1]);
        position.addOption("Bottom", SCORE_POSITIONS[2]);
        position.addOption("Really Bottom", SCORE_POSITIONS[3]);
        position.addOption("Really Really Bottom", SCORE_POSITIONS[3]);
    }

    public Pose2d getSelected(){
        return position.get();
    }

    
}
