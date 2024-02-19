// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team6328.util;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

@java.lang.SuppressWarnings({"java:S1118", "java:S115", "java:S2386"})

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

  public static double blueWingX = Units.inchesToMeters(229.201);
  public static double redWingX = fieldLength - blueWingX;
  public static double bluePodiumX = Units.inchesToMeters(126.75);
  public static double redPodiumX = fieldLength - bluePodiumX;
  public static double blueStartingLineX = Units.inchesToMeters(74.111);
  public static double redStartingLineX = fieldLength - blueStartingLineX;

  public static Translation2d blueAmpCenter =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));
  public static Translation2d redAmpCenter = FieldConstants.flipForRedSide(blueAmpCenter);

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static double centerlineX = fieldLength / 2.0;

    // mechadv says they need to be updated but they look correct
    public static double centerlineFirstY = Units.inchesToMeters(29.638);
    private static double centerlineSeparationY = Units.inchesToMeters(66);
    private static double blueSpikeX = Units.inchesToMeters(114.0);
    // should be right
    private static double blueSpikeFirstY = Units.inchesToMeters(161.638);
    private static double blueSpikeSeparationY = Units.inchesToMeters(57);

    // Notes in center
    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] blueSpikeTranslations = new Translation2d[3];
    public static Translation2d[] redSpikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < blueSpikeTranslations.length; i++) {
        blueSpikeTranslations[i] =
            new Translation2d(blueSpikeX, blueSpikeFirstY + (i * blueSpikeSeparationY));
      }
    }

    static {
      for (int i = 0; i < redSpikeTranslations.length; i++) {
        redSpikeTranslations[i] = FieldConstants.flipForRedSide(blueSpikeTranslations[i]);
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class BlueSpeaker {

    /** Center of the speaker opening (blue alliance) */
    public static Pose2d blueCenterSpeakerOpening =
        new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());

    // corners (blue alliance origin)
    public static Translation3d blueTopRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(13.091));

    public static Translation3d blueTopLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static Translation3d blueBottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static Translation3d blueBottomLeftSpeaker =        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
  }

  public static final class RedSpeaker {
    /** Center of the speaker opening (blue alliance) */
    public static Pose2d redCenterSpeakerOpening =
        flipForRedSide(BlueSpeaker.blueCenterSpeakerOpening);

    // corners (red alliance origin)
    public static Translation3d redTopRightSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueTopLeftSpeaker);
    public static Translation3d redTopLeftSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueTopRightSpeaker);
    public static Translation3d redBottomRightSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueBottomLeftSpeaker);
    public static Translation3d redBottomLeftSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueBottomRightSpeaker);
  }


   public static class BlueChainAmpCenter{
    public static Translation2d blueChainAmp =  new Translation2d(Units.inchesToMeters(170.73), Units.inchesToMeters(197.885)); 
    public static Pose2d blueChainAmpPose = new Pose2d(blueChainAmp, new Rotation2d((2*Math.PI)/3)); 

  }

  public static class BlueChainSourceCenter{
    public static Translation2d blueChainSource = new Translation2d(Units.inchesToMeters(170.73), Units.inchesToMeters(125.405)); 
    public static Pose2d blueChainSourcePose = new Pose2d(blueChainSource, new Rotation2d(-(2*Math.PI)/3));
  }

   public static class BlueChainMiddleCenter{
    public static Translation2d blueChainMiddle =  new Translation2d(Units.inchesToMeters(233.48), Units.inchesToMeters(161.62)); 
    public static Pose2d blueChainMiddlePose = new Pose2d(blueChainMiddle, new Rotation2d(0));
  }

  public static class RedChainAmpCenter{
    public static Translation2d redChainAmp = flipForRedSide(BlueChainAmpCenter.blueChainAmp);
    public static Pose2d redChainAmpPose = flipForRedSide(BlueChainAmpCenter.blueChainAmpPose); 
  }

  public static class RedChainSourceCenter{
    public static Translation2d redChainSource = flipForRedSide(BlueChainSourceCenter.blueChainSource);
    public static Pose2d redChainSourcePose = flipForRedSide(BlueChainSourceCenter.blueChainSourcePose);
  }

   public static class RedChainMiddleCenter{
    public static Translation2d redChainMiddle = flipForRedSide(BlueChainMiddleCenter.blueChainMiddle);
    public static Pose2d redChainMiddlePose = flipForRedSide(BlueChainMiddleCenter.blueChainMiddlePose); 
  }




  public static class RedSource{
    public static Translation2d redSourceCenter = new Translation2d(Units.inchesToMeters(35.6247), Units.inchesToMeters(21.9704)); //Change coordinates to be 17 centimeters in front of the chain
    public static Pose2d redSourcePose = new Pose2d(redSourceCenter, new Rotation2d(2.8163-((Math.PI)/2)));
  }


  public static class BlueSource{
    public static Translation2d blueSourceCenter = flipForRedSide(RedSource.redSourceCenter);
    public static Pose2d blueSourcePose = flipForRedSide(RedSource.redSourcePose);
  }

  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static Translation3d flipForRedSide(Translation3d translation) {
    return new Translation3d(
        fieldLength - translation.getX(), translation.getY(), translation.getZ());
  }

  public static Translation2d flipForRedSide(Translation2d translation) {
    return new Translation2d(fieldLength - translation.getX(), translation.getY());
  }

  public static Pose2d flipForRedSide(Pose2d pose) {
    return new Pose2d(
        fieldLength - pose.getX(), pose.getY(), new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

  public static Rotation2d flipForRedSide(Rotation2d rotation) {
    return new Rotation2d(Math.PI).minus(rotation);
  }
}
