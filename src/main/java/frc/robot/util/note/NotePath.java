package frc.robot.util.note;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;

public class NotePath {
    private ArrayList<Pose3d> pathPosition;
    private ArrayList<PathPoint> pathPoints;
    private NoteShot shot;
    private double totalTime;

    private double gravity = 9.81;

    public NotePath(NoteShot shot) {
        this.shot = shot;
        this.pathPosition = new ArrayList<Pose3d>();
        this.pathPoints = new ArrayList<PathPoint>();
        this.pathPosition.add(shot.getShotPosition());
        this.pathPoints.add(shot.getFirstPoint());
    }

    public Pose3d[] getPathPosition() {
        return pathPosition.toArray(new Pose3d[0]);
    }

    public void generatePath() {
        double time = 0;
        double dt = 0.01;
        PathPoint pathPoint = shot.getFirstPoint();
        
        double ax = 0;
        double ay = 0;
        double az = -gravity;
        while (pathPoint.getPosition().getZ() >= 0) {
            time += dt;

            double x = pathPoint.getPosition().getX();
            double y = pathPoint.getPosition().getY();
            double z = pathPoint.getPosition().getZ();
            
            double vx = pathPoint.getDx();
            double vy = pathPoint.getDy();
            double vz = pathPoint.getDz();
            
            x += vx * dt + 0.5 * ax * dt * dt;
            y += vy * dt + 0.5 * ay * dt * dt;
            z += vz * dt + 0.5 * az * dt * dt;
            vx += ax * dt;
            vy += ay * dt;
            vz += az * dt;

            Pose3d nextPose = new Pose3d(x, y, z, shot.getShotPosition().getRotation());
            pathPoint = new PathPoint(nextPose, vx, vy, vz, time);
            
            pathPoints.add(pathPoint);
            pathPosition.add(nextPose);
        }
        
        totalTime = time;
        Logger.recordOutput("NotePath", getPathPosition());
    }

    public Pose3d samplePathPoint(double time) {
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            if (pathPoints.get(i).getTime() > time) {
                // interpolate between the point and the next one
                PathPoint prev = pathPoints.get(i - 1);
                PathPoint next = pathPoints.get(i);
                double alpha = (time - prev.getTime()) / (next.getTime() - prev.getTime());
                return prev.getPosition().interpolate(next.getPosition(), alpha);
            }
        }
        return pathPoints.get(pathPoints.size() - 1).getPosition();
    }

    public double getTotalTime() {
        return totalTime;
    }
        
}
