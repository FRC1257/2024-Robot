package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;

public class VisionIOPhoton implements VisionIO {

    private final PhotonCamera backLeft;
    private final PhotonPoseEstimator backLeftEstimator;

    // private final PhotonCamera orangeCamera;
    // private final PhotonPoseEstimator orangeEstimator;

    private final PhotonCamera noteCamera;
    // private final PhotonPoseEstimator orangeEstimator2;
    // private boolean noteCameraObjectMode = false;
    //private final PhotonPoseEstimator noteEstimator;

    private Pose2d lastEstimate = new Pose2d();
    
    public VisionIOPhoton() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        //These directions are arbitrary for now
        //Back Left
        backLeft = new PhotonCamera(kRaspberryCameraName2);
        backLeftEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeft, kRaspberryRobotToCam);
        backLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        noteCamera = new PhotonCamera(kNoteCameraName);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;

        PhotonPipelineResult[] results = getAprilTagResults();
        PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);
        
        // add code to check if the closest target is in front or back
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        Optional<Pose2d> averageEst = getAverageEstimate(results, photonEstimators);
        inputs.timestamp = estimateLatestTimestamp(results);

        if (averageEst.isPresent()) {
            inputs.estimate = averageEst.get();
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
            inputs.tagCount = tagCounts(results);
            inputs.hasEstimate = true;
        } else {
            inputs.timestamp = inputs.timestamp;
            inputs.hasEstimate = false;
        }

        // get note data
        //all note information is gotten here
        //just need to do something with this information
        var note_result = getLatestResult(noteCamera);
        
        inputs.noteTimestamp = note_result.getTimestampSeconds();
        inputs.noteConfidence = new double[note_result.getTargets().size()];
        inputs.notePitch = new double[note_result.getTargets().size()];
        inputs.noteYaw = new double[note_result.getTargets().size()];
        inputs.noteArea = new double[note_result.getTargets().size()];
        for (int i = 0; i < note_result.getTargets().size(); i++) {
            // inputs.noteConfidence[i] = note_result.getTargets().get(i).getConfidence();
            inputs.notePitch[i] = note_result.getTargets().get(i).getPitch();
            inputs.noteYaw[i] = note_result.getTargets().get(i).getYaw();
            inputs.noteArea[i] = note_result.getTargets().get(i).getArea();
        }
        
        Logger.recordOutput("Vision/BackLeftConnected", backLeft.isConnected());
    }

    private PhotonPipelineResult[] getAprilTagResults() {
        PhotonPipelineResult front_result2 = getLatestResult(backLeft);
        Logger.recordOutput("Vision/frontleft", front_result2.getTargets().size());
        return new PhotonPipelineResult[] { front_result2 };
    }

    
    private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
        backLeftEstimator.setReferencePose(currentEstimate);
        return new PhotonPoseEstimator[] { backLeftEstimator};
    }

    private int tagCounts(PhotonPipelineResult[] results) {
        int tags = 0;
        for (PhotonPipelineResult result : results) {
            tags += result.targets.size();
        }
        return tags;
    }

    @Override
    public Translation2d calculateNoteTranslation(VisionIOInputs inputs) {
        return null;
    }

    @Override
    public Pose2d calculateNotePose(Pose2d robotPose, Translation2d noteTranslation){
        return new Pose2d(robotPose.getX() + noteTranslation.getX(), robotPose.getY() + noteTranslation.getY(), robotPose.getRotation());
    }

    @Override
    public Rotation2d getAngleToNote() {
        return null;
    }

    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets()/*   && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD *//* 
                && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE */;
    }

    @Override
    public void setNoteCameraObjectMode(boolean mode) {
        return;
    }
}
