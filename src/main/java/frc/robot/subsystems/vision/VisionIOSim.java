package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class VisionIOSim implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonCamera backCamera;
    private final PhotonCamera camera2;
    private final PhotonCamera noteCamera;

    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator backPhotonEstimator;
    private final PhotonPoseEstimator photonEstimator2;

    // Simulation
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim backCameraSim;
    private PhotonCameraSim cameraSim2;
    private PhotonCameraSim noteCameraSim;
    private VisionSystemSim visionSim;

    private Pose2d lastEstimate = new Pose2d();

    public VisionIOSim() {
        camera = new PhotonCamera(kRaspberryCameraName);
        backCamera = new PhotonCamera(kOrangeCameraName);
        camera2 = new PhotonCamera(kRaspberryCameraName2);
        noteCamera = new PhotonCamera(kNoteCameraName);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                kRaspberryRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPhotonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera,
                kOrangeRobotToCam);
        backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2,
                kRaspberryRobotToCam);
        photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual
        // camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        backCameraSim = new PhotonCameraSim(backCamera, cameraProp);
        cameraSim2 = new PhotonCameraSim(camera2, cameraProp);
        noteCameraSim = new PhotonCameraSim(noteCamera, cameraProp);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, kRaspberryRobotToCam);
        visionSim.addCamera(backCameraSim, kOrangeRobotToCam);
        visionSim.addCamera(cameraSim2, kRaspberryRobotToCam2);
        visionSim.addCamera(noteCameraSim, kNoteRobotToCam);

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
        PhotonPipelineResult[] results = { front_result, back_result, front_result2 };
        PhotonPoseEstimator[] photonEstimators = { photonEstimator, backPhotonEstimator, photonEstimator2 };

        inputs.estimate = currentEstimate;

        // add code to check if the closest target is in front or back
        Optional<Pose2d> averageEst = getAverageEstimate(results, photonEstimators);
        inputs.timestamp = estimateLatestTimestamp(results);

        if (averageEst.isPresent()) {
            inputs.estimate = averageEst.get();
            inputs.targets3d = getTargetsPositions(results);
            inputs.targets = Pose3dToPose2d(inputs.targets3d);
        }

        Logger.recordOutput("Vision/OrangeConnected", camera.isConnected());
        Logger.recordOutput("Vision/RaspberryConnected", backCamera.isConnected());
        Logger.recordOutput("Vision/NoteConnected", noteCamera.isConnected());
        Logger.recordOutput("Vision/Raspberry2Connected", camera2.isConnected());
        
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
    
}
