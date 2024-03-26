package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;

public class VisionIOPhoton implements VisionIO {

    private final PhotonCamera camera1;
    private final PhotonPoseEstimator camera1Estimator;

    private final PhotonCamera camera2;
    private final PhotonPoseEstimator camera2Estimator;

    private final PhotonCamera camera3;
    private final PhotonPoseEstimator camera3Estimator;

    private Pose2d lastEstimate = new Pose2d();
    
    public VisionIOPhoton() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        
        // Camera 1
        camera1 = new PhotonCamera(cam1Name);
        camera1Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, 
            cam1RobotToCam);
        camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Camera 2
        camera2 = new PhotonCamera(cam2Name);
        camera2Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, cam2RobotToCam);
        camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //Front Left
        camera3 = new PhotonCamera(cam3Name);
        camera3Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera3, cam3RobotToCam);
        camera3Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;

        PhotonPipelineResult[] results = getAprilTagResults();
        PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);
        
        // add code to check if the closest target is in front or back
        inputs.estimate = new Pose2d[0];

        // add code to check if the closest target is in front or back
        Optional<Pose2d> averageEst = getAverageEstimate(results, photonEstimators);
        inputs.timestamp = estimateLatestTimestamp(results);

        if (averageEst.isPresent()) {
            inputs.estimate = getEstimatesArray(results, photonEstimators);
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
            inputs.tagCount = tagCounts(results);
            inputs.hasEstimate = true;
            Logger.recordOutput("Vision/EstimateAverage", averageEst.get());
        } else {
            inputs.timestamp = inputs.timestamp;
            inputs.hasEstimate = false;
        }

        // Log if the robot code can see these cameras
        Logger.recordOutput("Vision/cam1/Connected", camera1.isConnected());
        Logger.recordOutput("Vision/cam2/Connected", camera2.isConnected());
        Logger.recordOutput("Vision/cam3/Connected", camera3.isConnected());
    }

    private PhotonPipelineResult[] getAprilTagResults() {
        PhotonPipelineResult cam1_result = getLatestResult(camera1);
        PhotonPipelineResult cam2_result = getLatestResult(camera2);
        PhotonPipelineResult cam3_result = getLatestResult(camera3);

        // Number of targets
        Logger.recordOutput("Vision/cam1/results", cam1_result.getTargets().size());
        Logger.recordOutput("Vision/cam2/results", cam2_result.getTargets().size());
        Logger.recordOutput("Vision/cam3/results", cam3_result.getTargets().size());

        // Pose Ambiguity
        Logger.recordOutput("Vision/cam1/PoseAmbiguity", cam1_result.getBestTarget().getPoseAmbiguity());
        Logger.recordOutput("Vision/cam2/PoseAmbiguity", cam2_result.getBestTarget().getPoseAmbiguity());
        Logger.recordOutput("Vision/cam3/PoseAmbiguity", cam3_result.getBestTarget().getPoseAmbiguity());

        // Target Yaw
        Logger.recordOutput("Vision/cam1/TargetYaw", cam1_result.getBestTarget().getYaw());
        Logger.recordOutput("Vision/cam2/TargetYaw", cam2_result.getBestTarget().getYaw());
        Logger.recordOutput("Vision/cam3/TargetYaw", cam3_result.getBestTarget().getYaw());

        return new PhotonPipelineResult[] { cam1_result, cam2_result, cam3_result };
    }

    private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
        camera1Estimator.setReferencePose(currentEstimate);
        camera2Estimator.setReferencePose(currentEstimate);
        camera3Estimator.setReferencePose(currentEstimate);

        return new PhotonPoseEstimator[] { camera1Estimator, camera2Estimator, camera3Estimator };
    }

    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD/* 
                && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE */;
    }

}
