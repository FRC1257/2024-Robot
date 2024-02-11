package frc.robot.util.note;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;

import static frc.robot.Constants.PivotArm.PivotArmSimConstants.kArmLength;

import org.littletonrobotics.junction.Logger;

public class NoteShot {
    private Pose3d shotPosition;
    private double shotStraightSpeed;
    private double shotTangentSpeed;

    public NoteShot(Pose3d shotPosition, double shotStraightSpeed, double shotTangentSpeed) {
        this.shotPosition = shotPosition;
        this.shotStraightSpeed = shotStraightSpeed;
        this.shotTangentSpeed = shotTangentSpeed;
    }

    public Pose3d getShotPosition() {
        return shotPosition;
    }

    public double getShotStraightSpeed() {
        return shotStraightSpeed;
    }

    public double getShotTangentSpeed() {
        return shotTangentSpeed;
    }

    public void setShotPosition(Pose3d shotPosition) {
        this.shotPosition = shotPosition;
    }

    public void setShotStraightSpeed(double shotStraightSpeed) {
        this.shotStraightSpeed = shotStraightSpeed;
    }

    public void setShotTangentSpeed(double shotTangentSpeed) {
        this.shotTangentSpeed = shotTangentSpeed;
    }

    public static NoteShot fromRobotInfo(Pose2d robotPose, Rotation2d pivotAngle, double shotLeftSpeed, double shotRightSpeed) {
        // calculate the position of the arm in 2d
        Pose2d armPose = robotPose.plus(new Transform2d(new Translation2d(0.098, robotPose.getRotation().plus(Rotation2d.fromDegrees(180))), new Rotation2d()));
        
        // get the 3d rotation of the arm
        Rotation3d rotation = new Rotation3d(0, pivotAngle.getRadians(), armPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians());
        Rotation3d noteRotation = new Rotation3d(0, Math.PI / 2, 0);
        
        Translation3d translation = new Translation3d(armPose.getTranslation().getX(), armPose.getTranslation().getY(), 0.28);
        Pose3d extendedPose = new Pose3d(translation, rotation);

        extendedPose = extendedPose.plus(new Transform3d(new Translation3d(-kArmLength, 0, 0), noteRotation));

        Logger.recordOutput("ExtendedPOse", extendedPose);

        double straightSpeed = (shotLeftSpeed + shotRightSpeed) / 2;
        double tangentSpeed = (shotLeftSpeed - shotRightSpeed) / 2;

        return new NoteShot(extendedPose, straightSpeed, tangentSpeed);
    }

    public PathPoint getFirstPoint() {
        // fix this code
        double dx = shotStraightSpeed * Math.cos(shotPosition.getRotation().getZ() + Math.PI) * Math.cos(shotPosition.getRotation().getY())
            /* + shotTangentSpeed * Math.cos(shotPosition.getRotation().getZ()) */;
        double dy = shotStraightSpeed * Math.sin(shotPosition.getRotation().getZ()) * Math.cos(shotPosition.getRotation().getY())
            /* + shotTangentSpeed * Math.sin(shotPosition.getRotation().getZ() + Math.PI / 2) */;
        double dz = shotStraightSpeed * Math.sin(shotPosition.getRotation().getY());
        
        return new PathPoint(
            shotPosition, 
            dx,
            dy,
            dz, 
            0
        );
    }

}