package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.cam1Name;
import static frc.robot.subsystems.vision.VisionConstants.cam1RobotToCam;
import static frc.robot.subsystems.vision.VisionConstants.cam2Name;
import static frc.robot.subsystems.vision.VisionConstants.cam2RobotToCam;
import static frc.robot.subsystems.vision.VisionConstants.cam3Name;
import static frc.robot.subsystems.vision.VisionConstants.cam3RobotToCam;
import static frc.robot.subsystems.vision.VisionConstants.kTagLayout;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIOPhoton implements VisionIO {

    private final PhotonCamera camera1;
    private final PhotonPoseEstimator camera1Estimator;

    private final PhotonCamera camera2;
    private final PhotonPoseEstimator camera2Estimator;

    private final PhotonCamera camera3;
    private final PhotonPoseEstimator camera3Estimator;

    private Pose2d lastEstimate = new Pose2d();

    LoggedDashboardBoolean killSideCams = new LoggedDashboardBoolean("Vision/KillSideCams", false);

    public VisionIOPhoton() {
        PortForwarder.add(5800, "photonvision.local", 5800);

        // Camera 1
        camera1 = new PhotonCamera(cam1Name);
        camera1Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1,
                cam1RobotToCam);
        camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Camera 2
        camera2 = new PhotonCamera(cam2Name);
        camera2Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2,
                cam2RobotToCam);
        camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Front Left
        camera3 = new PhotonCamera(cam3Name);
        camera3Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera3,
                cam3RobotToCam);
        camera3Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putBoolean("KillSideCams", false);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        lastEstimate = currentEstimate;

        PhotonPipelineResult[] results = getAprilTagResults();
        PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);

        inputs.estimate = new Pose2d[] { new Pose2d() };

        // add code to check if the closest target is in front or back
        inputs.timestamp = estimateLatestTimestamp(results);

        if (hasEstimate(results)) {
            // inputs.results = results;
            inputs.estimate = getEstimatesArray(results, photonEstimators);
            inputs.hasEstimate = true;

            int[][] cameraTargets = getCameraTargets(results);
            inputs.camera1Targets = cameraTargets[0];

            if (killSideCams.get()) {
                inputs.camera2Targets = new int[0];
                inputs.camera3Targets = new int[0];
            } else {
                inputs.camera2Targets = cameraTargets[1];
                inputs.camera3Targets = cameraTargets[2];
            }

            Pose3d[] tags = getTargetsPositions(results);
            Logger.recordOutput("Vision/Targets3D", tags);
            Logger.recordOutput("Vision/Targets", Pose3dToPose2d(tags));
            Logger.recordOutput("Vision/TagCounts", tagCounts(results));
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
        if (killSideCams.get()) {
            PhotonPipelineResult cam1_result = getLatestResult(camera1);

            printStuff("cam1", cam1_result);

            return new PhotonPipelineResult[] { cam1_result };
        }

        PhotonPipelineResult cam1_result = getLatestResult(camera1);
        PhotonPipelineResult cam2_result = getLatestResult(camera2);
        PhotonPipelineResult cam3_result = getLatestResult(camera3);

        printStuff("cam1", cam1_result);
        printStuff("cam2", cam2_result);
        printStuff("cam3", cam3_result);

        return new PhotonPipelineResult[] { cam1_result, cam2_result, cam3_result };
    }

    private void printStuff(String name, PhotonPipelineResult result) {
        Logger.recordOutput("Vision/" + name + "/results", result.getTargets().size());

        PhotonTrackedTarget target = result.getBestTarget();
        if (target != null) {
            Logger.recordOutput("Vision/" + name + "/PoseAmbiguity", result.getBestTarget().getPoseAmbiguity());
            Logger.recordOutput("Vision/" + name + "/Yaw", result.getBestTarget().getYaw());
        }
    }

    private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
        if (killSideCams.get()) {
            camera1Estimator.setReferencePose(currentEstimate);

            return new PhotonPoseEstimator[] { camera1Estimator };
        }

        camera1Estimator.setReferencePose(currentEstimate);
        camera2Estimator.setReferencePose(currentEstimate);
        camera3Estimator.setReferencePose(currentEstimate);

        return new PhotonPoseEstimator[] { camera1Estimator, camera2Estimator, camera3Estimator };
    }

    @Override
    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
        /*
         * && kTagLayout.
         * getTagPose(
         * result.
         * getBestTarget().
         * getFiducialId())
         * .get().toPose2d(
         * ).getTranslation
         * ()
         * .getDistance(
         * lastEstimate.
         * getTranslation()
         * ) < MAX_DISTANCE
         */;
    }
}
