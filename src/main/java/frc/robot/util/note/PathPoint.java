package frc.robot.util.note;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;

public class PathPoint {
    private Pose3d position;
    private double dx, dy, dz;
    private double time;

    public PathPoint(Pose3d position, double dx, double dy, double dz, double time) {
        this.position = position;
        this.dx = dx;
        this.dy = dy;
        this.dz = dz;
        this.time = time;
    }

    public Pose3d getPosition() {
        return position;
    }

    public Twist3d getVelocity() {
        return new Twist3d(dx, dy, dz, 0, 0, 0);
    }

    public double getTime() {
        return time;
    }

    public double getDx() {
        return dx;
    }

    public double getDy() {
        return dy;
    }

    public double getDz() {
        return dz;
    }
}
