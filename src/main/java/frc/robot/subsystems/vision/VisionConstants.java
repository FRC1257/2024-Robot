package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
        public static final String cam1Name = "April_Camera3";
        public static final String cam3Name = "";
        public static final String kNoteCameraName = "Note_Camera";
        public static final String cam2Name = "April_Camera2";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        private static double cameraPitchAngle = Rotation2d.fromDegrees(10).getRadians();
        private static double cameraYawAngle = Rotation2d.fromDegrees(185).getRadians();
        private static double cameraRollAngle = Rotation2d.fromDegrees(0).getRadians();
        public static final Transform3d cam1RobotToCam = new Transform3d(
                        new Translation3d(
                                        Units.inchesToMeters(-9),
                                        Units.inchesToMeters(7),
                                        Units.inchesToMeters(10)),
                        new Rotation3d(cameraRollAngle, cameraPitchAngle, cameraYawAngle));

        public static final Transform3d cam3RobotToCam = new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-9.5),
                                Units.inchesToMeters(7),
                                Units.inchesToMeters(4)
                        ),
                        new Rotation3d(cameraRollAngle, cameraPitchAngle, cameraYawAngle));
        public static final Transform3d cam2RobotToCam = new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-9.5),
                                Units.inchesToMeters(7),
                                Units.inchesToMeters(9.75)
                        ),
                        new Rotation3d(cameraRollAngle, cameraPitchAngle, cameraYawAngle));

        public static final int NOTE_PIPELINE = 0;
        public static final int NOTE_TAG_PIPELINE = 1;

        public static final double NoteCameraHeight = 6.0;

        public static final double NoteHeight = 0.0;

        // Note Camera Angle (used in sim)
        public static final Transform3d kNoteRobotToCam = new Transform3d(new Translation3d(0.5, 0.45, 0.23),
                        new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        public static final double AMBIGUITY_THRESHOLD = 0.5;
        public static final double MAX_DISTANCE = 4; // meters

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static Transform3d getSimVersion(Transform3d real) {
                return new Transform3d(
                        real.getTranslation(),
                        new Rotation3d(
                                0,
                                0,
                                real.getRotation().getZ()
                        )
                );
        }
}