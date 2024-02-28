package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.robot.Constants.Vision.*;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public Pose2d estimate = new Pose2d();
    public int tagCount = 0;
    public double timestamp = 0;
    public Pose2d[] targets = new Pose2d[0];
    public Pose3d[] targets3d = new Pose3d[0];

    // note detection
    public int notes = 0;
    public double noteTimestamp = 0;
    public double[] noteConfidence = new double[0];
    public double[] notePitch = new double[0];
    public double[] noteYaw = new double[0];
    public double[] noteSkew = new double[0];
    public double[] noteArea = new double[0];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {
  }

  public default PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    return camera.getLatestResult();
  }

  public default Optional<Pose2d> getAverageEstimate(PhotonPipelineResult[] results,
      PhotonPoseEstimator[] photonEstimator) {
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
    if (count == 0)
      return Optional.empty();
    Pose2d[] poseArray = new Pose2d[poses.size()];
    for (int i = 0; i < poses.size(); i++) {
      poseArray[i] = poses.get(i);
    }
    Logger.recordOutput("PoseEstimates", poseArray);
    return Optional.of(new Pose2d(x / count, y / count, new Rotation2d(rot / count)));
  }

  public default double estimateLatestTimestamp(PhotonPipelineResult[] results) {
    double latestTimestamp = 0;
    int count = 0;
    for (PhotonPipelineResult result : results) {
      latestTimestamp = result.getTimestampSeconds();
      count++;
    }
    return latestTimestamp / count;
  }

  public default boolean goodResult(PhotonPipelineResult result) {
    return true;
  }

  public default Pose3d[] getTargetsPositions(PhotonPipelineResult[] results) {
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

  public default Pose2d[] Pose3dToPose2d(Pose3d[] poses) {
    Pose2d[] pose2ds = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      pose2ds[i] = poses[i].toPose2d();
    }
    return pose2ds;
  }

  public default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult[] results,
      PhotonPoseEstimator[] photonEstimators) {
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

    if (numTags == 0)
      return estStdDevs;

    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = kMultiTagStdDevs;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }

  // Override this
  public default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    return kSingleTagStdDevs;
  }

  public default void NoteDetect() {
  } // the plan is to get the center of the ring, then set a PID loop turning it to
    // the center of the camera
}
