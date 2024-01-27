package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;

import static frc.robot.Constants.Vision.*;

import java.util.List;
import java.util.Optional;

public class VisionIOPhoton implements VisionIO {

    private final PhotonCamera raspberryCamera;
    private final PhotonPoseEstimator raspberryEstimator;

    private final PhotonCamera orangeCamera;
    private final PhotonPoseEstimator orangeEstimator;

    private final PhotonCamera noteCamera;

    private double lastEstTimestamp = 0;

    private PhotonPipelineResult latestRaspberryResult;
    private PhotonPipelineResult latestOrangeResult;
    
    public VisionIOPhoton() {
        raspberryCamera = new PhotonCamera(kRaspberryCameraName);
        raspberryEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, raspberryCamera, kRaspberryRobotToCam);
        raspberryEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        orangeCamera = new PhotonCamera(kOrangeCameraName);
        orangeEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, orangeCamera, kOrangeRobotToCam);
        orangeEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        noteCamera = new PhotonCamera(kNoteCameraName);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        raspberryEstimator.setReferencePose(currentEstimate);
        orangeEstimator.setReferencePose(currentEstimate);
        
        raspberryEstimator.setReferencePose(currentEstimate);
        orangeEstimator.setReferencePose(currentEstimate);
        
        latestRaspberryResult = getLatestResult(raspberryCamera);
        latestOrangeResult = getLatestResult(orangeCamera);
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        if (latestRaspberryResult.hasTargets() && latestOrangeResult.hasTargets()) {
            double front_ambiguity = latestRaspberryResult.getBestTarget().getPoseAmbiguity();
            double back_ambiguity = latestOrangeResult.getBestTarget().getPoseAmbiguity();
            if (front_ambiguity < back_ambiguity) {
                raspberryEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = latestRaspberryResult.getTimestampSeconds();
            } else {
                orangeEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = latestOrangeResult.getTimestampSeconds();
            }
        }
        else if (latestRaspberryResult.hasTargets()) {
            raspberryEstimator.update().ifPresent(est -> {
                inputs.estimate = est.estimatedPose.toPose2d();
            });
            lastEstTimestamp = latestRaspberryResult.getTimestampSeconds();
        } else if (latestOrangeResult.hasTargets()) {
            orangeEstimator.update().ifPresent(est -> {
                inputs.estimate = est.estimatedPose.toPose2d();
            });
            lastEstTimestamp = latestOrangeResult.getTimestampSeconds();
        }
        inputs.tagCount = latestRaspberryResult.getTargets().size() + latestOrangeResult.getTargets().size();
        inputs.timestamp = lastEstTimestamp;
        // inputs.stdDeviations = getEstimationStdDevs(inputs.estimate);
        List<PhotonTrackedTarget> front_tags = latestRaspberryResult.targets;
        List<PhotonTrackedTarget> back_tags = latestOrangeResult.targets;

        inputs.targets = new Pose2d[front_tags.size() + back_tags.size()];
        inputs.targets3d = new Pose3d[front_tags.size() + back_tags.size()];
        for (int i = 0; i < front_tags.size(); i++) {
            inputs.targets[i] = raspberryEstimator.getFieldTags().getTagPose(front_tags.get(i).getFiducialId()).get().toPose2d();
            inputs.targets3d[i] = raspberryEstimator.getFieldTags().getTagPose(front_tags.get(i).getFiducialId()).get();
        }
        for (int i=0; i < back_tags.size(); i++) {
            inputs.targets[i + front_tags.size()] = orangeEstimator.getFieldTags().getTagPose(back_tags.get(i).getFiducialId()).get().toPose2d();
            inputs.targets3d[i + front_tags.size()] = orangeEstimator.getFieldTags().getTagPose(back_tags.get(i).getFiducialId()).get();
        }

        // get note data
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
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // Check which camera has best estimate
        Optional<EstimatedRobotPose> orangeEst = orangeEstimator.update();
        Optional<EstimatedRobotPose> raspberryEst = raspberryEstimator.update();

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        double latestTimestamp = latestRaspberryResult.getTimestampSeconds();

        if (orangeEst.isPresent() && raspberryEst.isPresent()) {
            double orangeAmbiguity = latestRaspberryResult.getBestTarget().getPoseAmbiguity();
            double raspberryAmbiguity = latestOrangeResult.getBestTarget().getPoseAmbiguity();
            if (orangeAmbiguity < raspberryAmbiguity) {
                visionEst = orangeEst;
                latestTimestamp = latestOrangeResult.getTimestampSeconds();
            } else {
                visionEst = raspberryEst;
            }
        } else if (orangeEst.isPresent()) {
            visionEst = orangeEst;
            latestTimestamp = latestOrangeResult.getTimestampSeconds();
        } else if (raspberryEst.isPresent()) {
            visionEst = raspberryEst;
        }
        
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, PhotonPoseEstimator orangeEstimator) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult(camera).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = orangeEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        if (latestRaspberryResult.hasTargets() && latestOrangeResult.hasTargets())
            if (latestRaspberryResult.getBestTarget().getPoseAmbiguity() < latestOrangeResult.getBestTarget().getPoseAmbiguity())
                return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
            else return getEstimationStdDevs(estimatedPose, orangeCamera, orangeEstimator);
        else if (latestRaspberryResult.hasTargets())
            return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
        else if (latestOrangeResult.hasTargets())
            return getEstimationStdDevs(estimatedPose, orangeCamera, orangeEstimator);
        else return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
    }
}
