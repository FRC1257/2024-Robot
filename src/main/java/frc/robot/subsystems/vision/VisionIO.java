package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kMultiTagStdDevs;
import static frc.robot.subsystems.vision.VisionConstants.kSingleTagStdDevs;
import static frc.robot.subsystems.vision.VisionConstants.kTagLayout;

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

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public Pose2d[] estimate = new Pose2d[0];
    public double timestamp = 0;
    public double[] timestampArray = new double[0];

    public int[] camera1Targets = new int[0];
    public int[] camera2Targets = new int[0];
    public int[] camera3Targets = new int[0];

    public boolean hasEstimate = false;

    public byte[] results;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {
  }

  public default PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    return camera.getLatestResult();
  }

  public default Optional<Pose2d>[] getEstimates(PhotonPipelineResult[] results,
      PhotonPoseEstimator[] photonEstimator) {
    ArrayList<Optional<Pose2d>> estimates = new ArrayList<>();
    for (int i = 0; i < results.length; i++) {
      PhotonPipelineResult result = results[i];
      if (result.hasTargets()) {
        var est = photonEstimator[i].update();
        if (est.isPresent() && goodResult(result)) {
          estimates.add(Optional.of(est.get().estimatedPose.toPose2d()));
        } else {
          estimates.add(Optional.empty());
        }
      } else {
        estimates.add(Optional.empty());
      }
    }

    Optional<Pose2d>[] estimatesArray = estimates.toArray(new Optional[0]);
    return estimatesArray;
  }

  public default Pose2d[] getEstimatesArray(PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {
    Optional<Pose2d>[] estimates = getEstimates(results, photonEstimator);
    Pose2d[] estimatesArray = new Pose2d[estimates.length];
    for (int i = 0; i < estimates.length; i++) {
      if (estimates[i].isPresent() && estimates[i].get() != null) {
        estimatesArray[i] = estimates[i].get();
      }
    }

    int count = 0;
    for (int i = 0; i < estimatesArray.length; i++) {
      if (estimatesArray[i] != null) {
        count++;
      }
    }

    Pose2d[] finalEstimates = new Pose2d[count];
    int index = 0;
    for (int i = 0; i < estimatesArray.length; i++) {
      if (estimatesArray[i] != null) {
        finalEstimates[index] = estimatesArray[i];
        index++;
      }
    }

    return finalEstimates;
  }

  public default List<Matrix<N3, N1>> getStdArray(VisionIOInputs inputs, Pose2d currentPose) {
    List<Matrix<N3, N1>> stdsArray = new ArrayList<Matrix<N3, N1>>();

    for (int i = 0; i < getCameraTargets(inputs).length; i++) {
      if (getCameraTargets(inputs)[i].length != 0) {
        stdsArray.add(getEstimationStdDevs(inputs, currentPose, i));
      }
    }

    return stdsArray;
  }

  public default int[][] getCameraTargets(VisionIOInputs inputs) {
    return new int[][] { inputs.camera1Targets, inputs.camera2Targets, inputs.camera3Targets };
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
    return result.hasTargets();
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

  /**
   * The standard deviations of the estimated pose from
   * {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public default Matrix<N3, N1> getEstimationStdDevs(VisionIOInputs inputs, Pose2d pose, int camera) {
    var estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;
    int[] targets = getCameraTargets(inputs)[camera];
    for (var tgt : targets) {
      Optional<Pose3d> tagPose = kTagLayout.getTagPose(tgt);
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public default int[][] getCameraTargets(PhotonPipelineResult[] results) {
    int[][] targets = new int[results.length][];

    for (int i = 0; i < results.length; i++) {
      targets[i] = new int[results[i].targets.size()];
      for (int j = 0; j < results[i].targets.size(); j++) {
        targets[i][j] = results[i].targets.get(j).getFiducialId();
      }
    }

    return targets;
  }

  public default int tagCounts(PhotonPipelineResult[] results) {
    int tags = 0;
    for (PhotonPipelineResult result : results) {
      tags += result.targets.size();
    }
    return tags;
  }

  public default double[] getTimestampArray(PhotonPipelineResult[] results) {
    double[] timestamps = new double[results.length];
    for (int i = 0; i < results.length; i++) {
      timestamps[i] = results[i].getTimestampSeconds();
    }
    return timestamps;
  }

  public default boolean hasEstimate(PhotonPipelineResult[] results) {
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        return true;
      }
    }
    return false;
  }
}
