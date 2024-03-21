package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.drive.AllianceFlipUtil;


import org.littletonrobotics.junction.Logger;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.223);
  public static double fieldWidth = Units.inchesToMeters(323.277);
  public static double wingX = Units.inchesToMeters(229.201);
  public static double podiumX = Units.inchesToMeters(126.75);
  public static double startingLineX = Units.inchesToMeters(74.111);

  private static Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

      public static Pose2d TrapPose = new Pose2d(0,0, new Rotation2d(0)); // NEEDS TO BE ADJUSTED

  private static Pose2d ampPose = new Pose2d(ampCenter, Rotation2d.fromDegrees(-90));

  private static Pose2d pickupPose = new Pose2d(15.331, 1, Rotation2d.fromDegrees(-60));

  public static final class RobotConstants {
    public static double armLength = Units.inchesToMeters(20.5);
    public static double robotHeight = Units.inchesToMeters(11);
  }
  /** Staging locations for each note */
  public static final class StagingLocations {
    public static double centerlineX = Units.inchesToMeters(fieldLength / 2);

    // need to update
    public static double centerlineFirstY = Units.inchesToMeters(29.638);
    public static double centerlineSeparationY = Units.inchesToMeters(66);
    public static double spikeX = Units.inchesToMeters(114);
    // need
    public static double spikeFirstY = Units.inchesToMeters(161.638);
    public static double spikeSeparationY = Units.inchesToMeters(57);

    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
      Logger.recordOutput("centerlineTranslations", centerlineTranslations);
      Logger.recordOutput("spikeTranslations", spikeTranslations);
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    /** Center of the speaker opening (blue alliance) */
    public static Pose2d centerSpeakerOpening =
        new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());
  }

  // corners (blue alliance origin)
  public static Translation3d topRightSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(83.091));

  public static Translation3d topLeftSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

  public static Translation3d bottomRightSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
  public static Translation3d bottomLeftSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
  public static final Pose2d SpeakerPosition = new Pose2d(-0.2, (5 + 6.12)/2, new Rotation2d(0));
  public static final Pose3d SpeakerPosition3D = new Pose3d(SpeakerPosition).transformBy(new Transform3d(0,0, Units.inchesToMeters(100.324), new Rotation3d()));
  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private static Pose2d[] NOTE_POSITIONS = new Pose2d[] {
    new Pose2d(2.90,6.68,Rotation2d.fromDegrees(0)),
    new Pose2d(2.90,5.55,Rotation2d.fromDegrees(0)),
    new Pose2d(2.90,4.09,Rotation2d.fromDegrees(0)),
    new Pose2d(8.29,7.44,Rotation2d.fromDegrees(0)),
    new Pose2d(8.29,5.78,Rotation2d.fromDegrees(0)),
    new Pose2d(8.29,4.12,Rotation2d.fromDegrees(0)),
    new Pose2d(8.29,2.45,Rotation2d.fromDegrees(0)),
    new Pose2d(8.29,0.77,Rotation2d.fromDegrees(0)),
  };

  private static Pose2d[] START_POSITIONS = new Pose2d[] {
    new Pose2d(0.73,6.74,Rotation2d.fromDegrees(60)),
    new Pose2d(1.51,5.57,Rotation2d.fromDegrees(0)),
    new Pose2d(0.73,4.43,Rotation2d.fromDegrees(-60)),
    new Pose2d(0.73,3.25,Rotation2d.fromDegrees(0)),
    new Pose2d(0.73,2.27,Rotation2d.fromDegrees(0)),
  };

  private static Pose2d[] SCORE_POSITIONS = new Pose2d[] {
    new Pose2d(0.73,6.74,Rotation2d.fromDegrees(45)),
    new Pose2d(1.51,5.57,Rotation2d.fromDegrees(0)),
    new Pose2d(0.73,4.43,Rotation2d.fromDegrees(-45)),
    new Pose2d(4.02,5.82,Rotation2d.fromDegrees(0)),
    new Pose2d(2.61,3.46,Rotation2d.fromDegrees(0)),
    //some of these positions may need to change once we learn how far we can be and still shoot
  };

  // Flips a translation to the correct side of the field based on the current alliance color.
  public static Pose2d flippedPose(Pose2d pose) {
    return AllianceFlipUtil.apply(pose);
  }

  public static Translation2d flippedTranslation(Translation2d translation) {
    return AllianceFlipUtil.apply(translation);
  }

  public static Pose2d ampPose() {
    return flippedPose(ampPose);
  }

  public static Translation2d ampCenter() {
    return flippedTranslation(ampCenter);
  }
  
  public static Translation3d topRightSpeaker() {
    return AllianceFlipUtil.apply(topRightSpeaker);
  }

  public static Pose2d[] NOTE_POSITIONS() {
    return AllianceFlipUtil.apply(NOTE_POSITIONS);
  }

  public static Pose2d[] START_POSITIONS() {
    return AllianceFlipUtil.apply(START_POSITIONS);
  }

  public static Pose2d[] SCORE_POSITIONS() {
    return AllianceFlipUtil.apply(SCORE_POSITIONS);
  }

  public static Pose2d speakerPosition() {
    return AllianceFlipUtil.apply(SpeakerPosition);
  }

  public static Pose3d speakerPosition3D() {
    Pose2d flippedPose = flippedPose(SpeakerPosition);
    return new Pose3d(
      flippedPose.getTranslation().getX(),
      flippedPose.getTranslation().getY(),
      SpeakerPosition3D.getTranslation().getZ(),
      new Rotation3d(
        0,
        0,
        flippedPose.getRotation().getRadians()
      )
    );
  }

  public static Pose2d pickupPose() {
    return flippedPose(pickupPose);
  }
}