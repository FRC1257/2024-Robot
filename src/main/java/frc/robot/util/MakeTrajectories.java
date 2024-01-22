package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.Constants.DriveConstants.kPathConstraints;

public class MakeTrajectories {
    public static PathPlannerPath makeTrajectory(Pose2d startPose, Pose2d endPose) {
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        PathPlannerPath path = PathPlannerPath.fromPathPoints(
            List.of(
                new PathPoint(startPose.getTranslation(), new RotationTarget(0, startPose.getRotation()), kPathConstraints),
                new PathPoint(endPose.getTranslation(), new RotationTarget(0, endPose.getRotation()), kPathConstraints)
            ), 
            kPathConstraints, 
            new GoalEndState(0, endPose.getRotation())
        );

        return path;
    }
}
