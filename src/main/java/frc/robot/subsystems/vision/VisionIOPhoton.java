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
        
        var front_result = getLatestResult(raspberryCamera);
        var back_result = getLatestResult(orangeCamera);
        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        if (front_result.hasTargets() && back_result.hasTargets()) {
            double front_ambiguity = front_result.getBestTarget().getPoseAmbiguity();
            double back_ambiguity = back_result.getBestTarget().getPoseAmbiguity();
            if (front_ambiguity < back_ambiguity) {
                raspberryEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = front_result.getTimestampSeconds();
            } else {
                orangeEstimator.update().ifPresent(est -> {
                    inputs.estimate = est.estimatedPose.toPose2d();
                });
                lastEstTimestamp = back_result.getTimestampSeconds();
            }
        }
        else if (front_result.hasTargets()) {
            raspberryEstimator.update().ifPresent(est -> {
                inputs.estimate = est.estimatedPose.toPose2d();
            });
            lastEstTimestamp = front_result.getTimestampSeconds();
        } else if (back_result.hasTargets()) {
            orangeEstimator.update().ifPresent(est -> {
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

        if (orangeEst.isPresent() && raspberryEst.isPresent()) {
            var front_result = getLatestResult(raspberryCamera);
            var back_result = getLatestResult(orangeCamera);

            double orangeAmbiguity = front_result.getBestTarget().getPoseAmbiguity();
            double raspberryAmbiguity = back_result.getBestTarget().getPoseAmbiguity();
            if (orangeAmbiguity < raspberryAmbiguity) {
                visionEst = orangeEst;
            } else {
                visionEst = raspberryEst;
            }
        } else if (orangeEst.isPresent()) {
            visionEst = orangeEst;
        } else if (raspberryEst.isPresent()) {
            visionEst = raspberryEst;
        }

        double latestTimestamp = raspberryCamera.getLatestResult().getTimestampSeconds();
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
        if (getLatestResult(raspberryCamera).hasTargets() && getLatestResult(orangeCamera).hasTargets())
            if (getLatestResult(raspberryCamera).getBestTarget().getPoseAmbiguity() < getLatestResult(orangeCamera).getBestTarget().getPoseAmbiguity())
                return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
            else return getEstimationStdDevs(estimatedPose, orangeCamera, orangeEstimator);
        else if (getLatestResult(raspberryCamera).hasTargets())
            return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
        else if (getLatestResult(orangeCamera).hasTargets())
            return getEstimationStdDevs(estimatedPose, orangeCamera, orangeEstimator);
        else return getEstimationStdDevs(estimatedPose, raspberryCamera, orangeEstimator);
    }
}
