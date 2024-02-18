package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.Vision.*;

public class VisionIOSim implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonCamera backCamera;
    private final PhotonCamera camera2;

    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator backPhotonEstimator;
    private final PhotonPoseEstimator photonEstimator2;

    private double lastEstTimestamp = 0;

    // Simulation
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim backCameraSim;
    private PhotonCameraSim cameraSim2;
    private VisionSystemSim visionSim;

    private Pose2d lastEstimate = new Pose2d();

    public VisionIOSim() {
        camera = new PhotonCamera(kRaspberryCameraName);
        backCamera = new PhotonCamera(kOrangeCameraName);
        camera2 = new PhotonCamera(kRaspberryCameraName2);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRaspberryRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPhotonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, kOrangeRobotToCam);
        backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, kRaspberryRobotToCam);
        photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        backCameraSim = new PhotonCameraSim(backCamera, cameraProp);
        cameraSim2 = new PhotonCameraSim(camera2, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, kRaspberryRobotToCam);
        visionSim.addCamera(backCameraSim, kOrangeRobotToCam);
        visionSim.addCamera(cameraSim2, kRaspberryRobotToCam2);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;
        visionSim.update(currentEstimate);
        photonEstimator.setReferencePose(currentEstimate);
        backPhotonEstimator.setReferencePose(currentEstimate);
        photonEstimator2.setReferencePose(currentEstimate);
        
        PhotonPipelineResult front_result = getLatestResult(camera);
        PhotonPipelineResult back_result = getLatestResult(backCamera);
        PhotonPipelineResult front_result2 = getLatestResult(camera2);
        PhotonPipelineResult[] results = {front_result, back_result, front_result2};
        PhotonPoseEstimator[] photonEstimators = {photonEstimator, backPhotonEstimator, photonEstimator2};
        
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        Optional<Pose2d> averageEst = getAverageEstimate(results, photonEstimators);
        inputs.timestamp = estimateLatestTimestamp(results);
        
        if (averageEst.isPresent()) {
            inputs.estimate = averageEst.get();
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
        } else {
            inputs.timestamp = lastEstTimestamp;
        }
    }

    public Optional<Pose2d> getAverageEstimate(PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {
        double x = 0;
        double y = 0;
        double rot = 0;
        int count = 0;
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        for (int i = 0; i < results.length; i++) {
            PhotonPipelineResult result = results[i];
            if (result.hasTargets()) {
                var est = photonEstimator[i].update();
                if (est.isPresent() && goodResult(result)) {
                    x += est.get().estimatedPose.getTranslation().getX();
                    y += est.get().estimatedPose.getTranslation().getY();
                    rot += est.get().estimatedPose.getRotation().getAngle();
                    poses.add(est.get().estimatedPose.toPose2d());
                    count++;
                }
            }
        }
        if (count == 0) return Optional.empty();
        Pose2d[] poseArray = new Pose2d[poses.size()];
        for (int i = 0; i < poses.size(); i++) {
            poseArray[i] = poses.get(i);
        }
        Logger.recordOutput("PoseEstimates", poseArray);
        return Optional.of(new Pose2d(x / count, y / count, new Rotation2d(rot / count)));
    }

    public double estimateLatestTimestamp(PhotonPipelineResult[] results) {
        double latestTimestamp = 0;
        int count = 0;
        for (PhotonPipelineResult result : results) {
                latestTimestamp = result.getTimestampSeconds();
                count++;
            
        }
        return latestTimestamp / count;
    }

    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation().getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE;
    }

    public Pose3d[] getTargetsPositions(PhotonPipelineResult[] results) {
        int total_targets = 0;
        for (int i = 0; i < results.length; i++) {
            if (goodResult(results[i])) {
                total_targets += results[i].getTargets().size();
            }
        }
        Pose3d[] targets = new Pose3d[total_targets];
        int index = 0;
        for (int i = 0; i < results.length; i++) {
            if (goodResult(results[i])) {
                for (PhotonTrackedTarget target : results[i].getTargets()) {
                    targets[index] = kTagLayout.getTagPose(target.getFiducialId()).get();
                    index++;
                }
            }
        }
        return targets;
    }

    public Pose2d[] Pose3dToPose2d(Pose3d[] poses) {
        Pose2d[] pose2ds = new Pose2d[poses.length];
        for (int i = 0; i < poses.length; i++) {
            pose2ds[i] = poses[i].toPose2d();
        }
        return pose2ds;
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimators) {
        var estStdDevs = kSingleTagStdDevs;
        double avgDist = 0;
        int numTags = 0;

        for (int i = 0; i < results.length; i++) {
            PhotonPipelineResult result = results[i];
            PhotonPoseEstimator photonEstimator = photonEstimators[i];

            if (goodResult(result)) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                
                for (PhotonTrackedTarget tgt : targets) {
                    var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    
                    if (tagPose.isPresent()) {
                        numTags++;
                        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
                    }
                }
            }
        }

        if (numTags == 0) return estStdDevs;
        
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        PhotonPipelineResult frontResult = getLatestResult(camera);
        PhotonPipelineResult backResult = getLatestResult(backCamera);
        PhotonPipelineResult frontResult2 = getLatestResult(camera2);

        PhotonPipelineResult[] results = {frontResult, backResult, frontResult2};
        PhotonPoseEstimator[] estimators = {photonEstimator, backPhotonEstimator, photonEstimator2};

        if (goodResult(frontResult) || goodResult(backResult) || goodResult(frontResult2)) {
            return getEstimationStdDevs(estimatedPose, results, estimators);
        } else {
            return kSingleTagStdDevs; // or any default value
        }
    }

}
