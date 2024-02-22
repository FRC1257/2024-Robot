package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class NoteChooser {

    private String name;
    private LoggedDashboardChooser position;

    public NoteChooser(String name){
        this.name = name;
    }

    private void setNotePosition(){
        position.addOption(name, name);
    }

    private void setStartPosition(){
        position.addOption("Top", );
    }

    private void setScorePosition(){
        position.addOption(name, name);
    }
}
