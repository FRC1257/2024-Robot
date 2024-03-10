package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;

public class NoteChooser {

    private SendableChooser<Pose2d> position;

    public NoteChooser(String name){
        position = new SendableChooser<Pose2d>();
        SmartDashboard.putData(name, position);
    }

    public void setNotePosition(){
        Pose2d[] NOTE_POSITIONS = FieldConstants.NOTE_POSITIONS();
        position.setDefaultOption("Note 1", NOTE_POSITIONS[0]);
        for (int i = 1; i < NOTE_POSITIONS.length; i++){
            position.addOption("Note " + (i + 1), NOTE_POSITIONS[i]);
        }
    }

    public void setStartPosition(){
        Pose2d[] START_POSITIONS = FieldConstants.START_POSITIONS();
        position.setDefaultOption("Top", START_POSITIONS[0]);
        position.addOption("Center", START_POSITIONS[1]);
        position.addOption("Bottom", START_POSITIONS[2]);
        position.addOption("Really Bottom", START_POSITIONS[3]);
        position.addOption("Really Really Bottom", START_POSITIONS[3]);
    }

    public void setScorePosition(){
        Pose2d[] SCORE_POSITIONS = FieldConstants.SCORE_POSITIONS();
        position.setDefaultOption("Top", SCORE_POSITIONS[0]);
        position.addOption("Center", SCORE_POSITIONS[1]);
        position.addOption("Bottom", SCORE_POSITIONS[2]);
        position.addOption("Really Bottom", SCORE_POSITIONS[3]);
        position.addOption("Really Really Bottom", SCORE_POSITIONS[3]);
    }

    public Pose2d getSelected(){
        Pose2d thing = position.getSelected();

        if (thing == null){
            System.out.println("Null value for some reason");
            thing = FieldConstants.NOTE_POSITIONS()[0];
        }

        return thing;
    }

    
}
//     // Add commadns to the list
//commands.add(drive.goPose(AutoChooser.NoteOneChooser.getSelected()).alongWith(intake)); // add intake stuff here too 
// commmands.add(intakeWhile); // use vision to detect the note 
//commands.add(drive.goPose(AutoChooser.NoteOneShotChooser.getSelected()));
//commands.add(shoot);