package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class VisionIOSim implements VisionIO {
    private final PhotonCamera cam1;
    private final PhotonCamera cam2;
    private final PhotonCamera cam3;

    private final PhotonPoseEstimator cam1Estimator;
    private final PhotonPoseEstimator cam2Estimator;
    private final PhotonPoseEstimator cam3Estimator;

    // Simulation
    private PhotonCameraSim cam1Sim;
    private PhotonCameraSim cam2Sim;
    private PhotonCameraSim cam3Sim;

    private VisionSystemSim visionSim;

    private Pose2d lastEstimate = new Pose2d();

    public VisionIOSim() {
        cam1 = new PhotonCamera(cam1Name);
        cam2 = new PhotonCamera(cam2Name);
        cam3 = new PhotonCamera(cam3Name);

        cam1Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1,
                getSimVersion(cam1RobotToCam));
        cam1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        cam2Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam2,
                getSimVersion(cam2RobotToCam));
        cam2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        cam3Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam3,
                getSimVersion(cam3RobotToCam));
        cam3Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create the vision system simulation which handles cam1s and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(kTagLayout);
        // Create simulated cam1 properties. These can be set to mimic your actual
        // cam1.
        var cam1Prop = new SimCameraProperties();
        cam1Prop.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cam1Prop.setCalibError(0.35, 0.10);
        cam1Prop.setFPS(15);
        cam1Prop.setAvgLatencyMs(50);
        cam1Prop.setLatencyStdDevMs(15);

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible
        // targets.
        cam1Sim = new PhotonCameraSim(cam1, cam1Prop);
        cam2Sim = new PhotonCameraSim(cam2, cam1Prop);
        cam3Sim = new PhotonCameraSim(cam3, cam1Prop);

        // Add the simulated cam1 to view the targets on this simulated field.
        visionSim.addCamera(cam1Sim, getSimVersion(cam1RobotToCam));
        visionSim.addCamera(cam2Sim, getSimVersion(cam2RobotToCam));
        visionSim.addCamera(cam3Sim, getSimVersion(cam3RobotToCam));

        cam1Sim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;
        visionSim.update(currentEstimate);
        cam1Estimator.setReferencePose(currentEstimate);
        cam2Estimator.setReferencePose(currentEstimate);
        cam3Estimator.setReferencePose(currentEstimate);

        PhotonPipelineResult front_result = getLatestResult(cam1);
        PhotonPipelineResult back_result = getLatestResult(cam2);
        PhotonPipelineResult front_result2 = getLatestResult(cam3);
        
        PhotonPipelineResult[] results = { front_result, back_result, front_result2 };
        PhotonPoseEstimator[] photonEstimators = { cam1Estimator, cam2Estimator, cam3Estimator };

        inputs.estimate = new Pose2d[] { new Pose2d() };

        // add code to check if the closest target is in front or back
        inputs.timestamp = estimateLatestTimestamp(results);

        if (hasEstimate(results)) {
            inputs.estimate = getEstimatesArray(results, photonEstimators);
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
            inputs.tagCount = tagCounts(results);
            inputs.hasEstimate = true;
        } else {
            inputs.timestamp = inputs.timestamp;
            inputs.hasEstimate = false;
        }

        Logger.recordOutput("Vision/OrangeConnected", cam1.isConnected());
        Logger.recordOutput("Vision/RaspberryConnected", cam2.isConnected());
        Logger.recordOutput("Vision/Raspberry2Connected", cam3.isConnected());
        
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
                && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE;
    }
 
}
