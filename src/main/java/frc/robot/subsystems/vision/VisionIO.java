package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d estimate = new Pose2d();
    public int tagCount = 0;
    public double timestamp = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {}

  public default PhotonPipelineResult getLatestResult() { return null; }

  public default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) { return null; }

  public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() { return null; }
}
