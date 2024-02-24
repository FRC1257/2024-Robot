package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Constants.Vision.*;

import java.util.Optional;

public class VisionIOPhoton implements VisionIO {

    private final PhotonCamera raspberryCamera;
    private final PhotonPoseEstimator raspberryEstimator;

    private final PhotonCamera raspberryCamera2;
    private final PhotonPoseEstimator raspberryEstimator2;

    private final PhotonCamera orangeCamera;
    private final PhotonPoseEstimator orangeEstimator;

    private final PhotonCamera noteCamera;
    private final PhotonPoseEstimator orangeEstimator2;
    //private final PhotonPoseEstimator noteEstimator;

    private Pose2d lastEstimate = new Pose2d();
    
    public VisionIOPhoton() {
        //These directions are arbitrary for now

        //Front Right
        raspberryCamera = new PhotonCamera(kRaspberryCameraName);
        raspberryEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, raspberryCamera, kRaspberryRobotToCam);
        raspberryEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Front Left
        raspberryCamera2 = new PhotonCamera(kRaspberryCameraName2);
        raspberryEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, raspberryCamera, kRaspberryRobotToCam2);
        raspberryEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Back Left
        orangeCamera = new PhotonCamera(kOrangeCameraName);
        orangeEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, orangeCamera, kOrangeRobotToCam);
        orangeEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Back Right
        noteCamera = new PhotonCamera(kNoteCameraName);
        orangeEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, orangeCamera, kOrangeRobotToCam);
        orangeEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //used for estimating pose based off 


       
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;
        //whether a camera looks for apriltags or notes is set in the website interface
        raspberryEstimator.setReferencePose(currentEstimate);
        orangeEstimator.setReferencePose(currentEstimate);
        raspberryEstimator2.setReferencePose(currentEstimate);

        PhotonPipelineResult front_result = getLatestResult(raspberryCamera);
        PhotonPipelineResult back_result = getLatestResult(orangeCamera);
        PhotonPipelineResult front_result2 = getLatestResult(raspberryCamera2);

        PhotonPipelineResult[] results = { front_result, back_result, front_result2 };
        PhotonPoseEstimator[] photonEstimators = { raspberryEstimator, orangeEstimator, raspberryEstimator2 };
        
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

        Logger.recordOutput("Vision/OrangeConnected", orangeCamera.isConnected());
        Logger.recordOutput("Vision/RaspberryConnected", raspberryCamera.isConnected());
        Logger.recordOutput("Vision/NoteConnected", noteCamera.isConnected());
        Logger.recordOutput("Vision/Raspberry2Connected", raspberryCamera2.isConnected());
    }



    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
                && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE;
    }
}
