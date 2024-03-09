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

    private final PhotonCamera raspberryCamera;
    private final PhotonPoseEstimator raspberryEstimator;

    // private final PhotonCamera raspberryCamera2;
    // private final PhotonPoseEstimator raspberryEstimator2;

    // private final PhotonCamera orangeCamera;
    // private final PhotonPoseEstimator orangeEstimator;

    private final PhotonCamera noteCamera;
    private final PhotonPoseEstimator orangeEstimator2;
    private boolean noteCameraObjectMode = false;
    //private final PhotonPoseEstimator noteEstimator;

    private Pose2d lastEstimate = new Pose2d();
    
    public VisionIOPhoton() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        //These directions are arbitrary for now

        //Front Right
        raspberryCamera = new PhotonCamera(kRaspberryCameraName);
        raspberryEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, raspberryCamera, kRaspberryRobotToCam);
        raspberryEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Front Left
        // raspberryCamera2 = new PhotonCamera(kRaspberryCameraName2);
        // raspberryEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, raspberryCamera, kRaspberryRobotToCam2);
        // raspberryEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Back Left
        // orangeCamera = new PhotonCamera(kOrangeCameraName);
        // orangeEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, orangeCamera, kOrangeRobotToCam);
        // orangeEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Back Right
        noteCamera = new PhotonCamera(kNoteCameraName);
        orangeEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, noteCamera, kOrangeRobotToCam);
        orangeEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //used for estimating pose based off 

    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;

        PhotonPipelineResult[] results = getAprilTagResults();
        PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);
        
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        Optional<Pose2d> averageEst = getAverageEstimate(results, photonEstimators);
        inputs.timestamp = estimateLatestTimestamp(results);

        if (averageEst.isPresent()) {
            inputs.estimate = averageEst.get();
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
        } else {
            inputs.timestamp = inputs.timestamp;
        }

        // get note data
        //all note information is gotten here
        //just need to do something with this information
        var note_result = getLatestResult(noteCamera);
        Logger.recordOutput("Vision/NoteCameraMode", noteCameraObjectMode);
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

        // Logger.recordOutput("Vision/OrangeConnected", orangeCamera.isConnected());
        Logger.recordOutput("Vision/RaspberryConnected", raspberryCamera.isConnected());
        Logger.recordOutput("Vision/NoteConnected", noteCamera.isConnected());
        Logger.recordOutput("Vision/NoteCameraPipeline", noteCamera.getPipelineIndex());
        // Logger.recordOutput("Vision/Raspberry2Connected", raspberryCamera2.isConnected());
    }

    private PhotonPipelineResult[] getAprilTagResults() {
        PhotonPipelineResult front_result = getLatestResult(raspberryCamera);
        // PhotonPipelineResult back_result = getLatestResult(orangeCamera);
        // PhotonPipelineResult front_result2 = getLatestResult(raspberryCamera2);

        if (!noteCameraObjectMode) {
            // return new PhotonPipelineResult[] { front_result, back_result, front_result2 };
            return new PhotonPipelineResult[] { front_result };
        } else {
            // return new PhotonPipelineResult[] { front_result, back_result, front_result2, getLatestResult(noteCamera) };
            return new PhotonPipelineResult[] { front_result, getLatestResult(noteCamera) };
        }
    }

    private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
        raspberryEstimator.setReferencePose(currentEstimate);
        // orangeEstimator.setReferencePose(currentEstimate);
        // raspberryEstimator2.setReferencePose(currentEstimate);

        if (!noteCameraObjectMode) {
            // return new PhotonPoseEstimator[] { raspberryEstimator, orangeEstimator, raspberryEstimator2 };
            return new PhotonPoseEstimator[] { raspberryEstimator };
        } else {
            orangeEstimator2.setReferencePose(currentEstimate);
            return new PhotonPoseEstimator[] { raspberryEstimator, orangeEstimator2 };
        }
    }

    @Override
    public Translation2d calculateNoteTranslation(VisionIOInputs inputs) {
        PhotonPipelineResult note_result = getLatestResult(noteCamera);
        //height of the note shouldn't matter, because ideally it's going to be on the ground
          if (note_result.hasTargets()) {
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                NoteCameraHeight, //need to set
                                NoteHeight, //should be 0 or the height of the note
                                0, //CAMERA_PITCH_RADIANS
                                Units.degreesToRadians(note_result.getBestTarget().getPitch()));            
                return PhotonUtils.estimateCameraToTargetTranslation(
                range, Rotation2d.fromDegrees(-note_result.getBestTarget().getYaw()));
        } else {
            return null;
        }
    }

    @Override
    public Pose2d calculateNotePose(Pose2d robotPose, Translation2d noteTranslation){
        return new Pose2d(robotPose.getX() + noteTranslation.getX(), robotPose.getY() + noteTranslation.getY(), robotPose.getRotation());
    }

    @Override
    public Rotation2d getAngleToNote() {
        PhotonPipelineResult note_result = getLatestResult(noteCamera);
        if (note_result.hasTargets()) {
            return Rotation2d.fromDegrees(-note_result.getBestTarget().getYaw());
        } else {
            return null;
        }
    }

    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
                && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE;
    }

    @Override
    public void setNoteCameraObjectMode(boolean mode) {
        noteCameraObjectMode = mode;
        if (noteCameraObjectMode) {
            noteCamera.setPipelineIndex(VisionConstants.NOTE_PIPELINE);
        } else {
            noteCamera.setPipelineIndex(VisionConstants.NOTE_TAG_PIPELINE);
        }
    }
}
