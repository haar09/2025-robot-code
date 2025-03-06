// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.AllianceFlipUtil;

import java.util.*;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = VisionConstants.kTagLayout.getFieldLength();
  public static final double fieldWidth = VisionConstants.kTagLayout.getFieldWidth();
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(
            VisionConstants.kTagLayout.getTagPose(16).get().getX(),
            0,
            Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftCenterFace =
        new Pose2d(
            rightCenterFace.getX(),
            fieldWidth - rightCenterFace.getY(),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> scoringPositions =
        new ArrayList<>(); // Starting at the right scoring facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> scoringPositions2d = new ArrayList<>();

    static {
      // Initialize faces
      var aprilTagLayout = VisionConstants.kTagLayout;
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      // Initialize scoring positions 11.5cm
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = level.xModifier;
          double adjustY = 0.1643;

          var rightScoringPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY-level.yModifier, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY-level.yModifier, new Rotation2d()))
                          .getY(),
                      1),
                  new Rotation3d(
                      0,
                      0,
                      poseDirection.getRotation().getRadians()+Units.degreesToRadians(level.angleModifier)));
          var leftScoringPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY-level.yModifier, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY-level.yModifier, new Rotation2d()))
                          .getY(),
                      1),
                  new Rotation3d(
                      0,
                      0,
                      poseDirection.getRotation().getRadians()+Units.degreesToRadians(level.angleModifier)));

          fillRight.put(level, rightScoringPose);
          fillLeft.put(level, leftScoringPose);
          fillRight2d.put(level, rightScoringPose.toPose2d());
          fillLeft2d.put(level, leftScoringPose.toPose2d());
        }
        scoringPositions.add(fillRight);
        scoringPositions.add(fillLeft);
        scoringPositions2d.add(fillRight2d);
        scoringPositions2d.add(fillLeft2d);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final double separation = Units.inchesToMeters(72.0);
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), fieldWidth / 2.0, new Rotation2d());
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() + separation, new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() - separation, new Rotation2d());
  }

  public enum ReefLevel {
    L1(1, 0, 0),
    L23(1.25, 0.115, -90);

    ReefLevel(double xModifier, double yModifier, double angleModifier) {
      this.xModifier = xModifier;
      this.yModifier = yModifier; // Degrees
      this.angleModifier = angleModifier;
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L23);
    }

    public final double xModifier;
    public final double yModifier;
    public final double angleModifier;
  }

  public static int findClosestReefside(Pose2d currentPose) {
    currentPose = AllianceFlipUtil.apply(currentPose);
    double minDistance = Double.MAX_VALUE;
    int closestIndex = -1;
    for (int i = 0; i < Reef.centerFaces.length; i++) {
      Pose2d candidate = Reef.centerFaces[i];
      if (candidate != null) {
        double distance = currentPose.getTranslation().getDistance(candidate.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          closestIndex = i;
        }
      }
    }
    return closestIndex;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;

  public record CoralObjective(int scoringId, ReefLevel reefLevel) {}

  public record AlgaeObjective(int id) {}
}