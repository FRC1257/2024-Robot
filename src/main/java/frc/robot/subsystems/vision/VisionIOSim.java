package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.List;
import java.util.Optional;
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
    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator backPhotonEstimator;
    private double lastEstTimestamp = 0;

    // Simulation
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim backCameraSim;
    private VisionSystemSim visionSim;

    public VisionIOSim() {
        camera = new PhotonCamera(kCameraName);
        backCamera = new PhotonCamera(kBackCameraName);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPhotonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, kBackRobotToCam);
        backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, kRobotToCam);
        visionSim.addCamera(backCameraSim, kBackRobotToCam);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        visionSim.update(currentEstimate);
        photonEstimator.setReferencePose(currentEstimate);
        backPhotonEstimator.setReferencePose(currentEstimate);
        
        var front_result = getLatestResult(camera);
        var back_result = getLatestResult(backCamera);
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        if (front_result.hasTargets() && back_result.hasTargets()) {
            double front_ambiguity = front_result.getBestTarget().getPoseAmbiguity();
            double back_ambiguity = back_result.getBestTarget().getPoseAmbiguity();
            if (front_ambiguity < back_ambiguity) {
                photonEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = front_result.getTimestampSeconds();
            } else {
                backPhotonEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = back_result.getTimestampSeconds();
            }
        }
        else if (front_result.hasTargets()) {
            photonEstimator.update().ifPresent(est -> {
                inputs.estimate = est.estimatedPose.toPose2d();
            });
            lastEstTimestamp = front_result.getTimestampSeconds();
        } else if (back_result.hasTargets()) {
            backPhotonEstimator.update().ifPresent(est -> {
                inputs.estimate = est.estimatedPose.toPose2d();
            });
            lastEstTimestamp = back_result.getTimestampSeconds();
        }
        inputs.tagCount = front_result.getTargets().size() + back_result.getTargets().size();
        inputs.timestamp = lastEstTimestamp;
        // inputs.stdDeviations = getEstimationStdDevs(inputs.estimate);
        List<PhotonTrackedTarget> front_tags = front_result.targets;
        List<PhotonTrackedTarget> back_tags = back_result.targets;

        inputs.targets = new Pose2d[front_tags.size() + back_tags.size()];
        inputs.targets3d = new Pose3d[front_tags.size() + back_tags.size()];
        for (int i = 0; i < front_tags.size(); i++) {
            inputs.targets[i] = photonEstimator.getFieldTags().getTagPose(front_tags.get(i).getFiducialId()).get().toPose2d();
            inputs.targets3d[i] = photonEstimator.getFieldTags().getTagPose(front_tags.get(i).getFiducialId()).get();
        }
        for (int i=0; i < back_tags.size(); i++) {
            inputs.targets[i + front_tags.size()] = backPhotonEstimator.getFieldTags().getTagPose(back_tags.get(i).getFiducialId()).get().toPose2d();
            inputs.targets3d[i + front_tags.size()] = backPhotonEstimator.getFieldTags().getTagPose(back_tags.get(i).getFiducialId()).get();
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        var visionEst = photonEstimator.update();
        double latestTimestamp = getLatestResult(camera).getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        visionEst.ifPresentOrElse(
                est ->
                        getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                () -> {
                    if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                });
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (getLatestResult(camera).hasTargets())
            return getEstimatedGlobalPose(camera, photonEstimator);
        else if (getLatestResult(backCamera).hasTargets())
            return getEstimatedGlobalPose(backCamera, photonEstimator);
        else return getEstimatedGlobalPose(camera, photonEstimator);
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult(camera).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        if (getLatestResult(camera).hasTargets())
            return getEstimationStdDevs(estimatedPose, camera, photonEstimator);
        else if (getLatestResult(backCamera).hasTargets())
            return getEstimationStdDevs(estimatedPose, backCamera, photonEstimator);
        else return getEstimationStdDevs(estimatedPose, camera, photonEstimator);
    }

}
