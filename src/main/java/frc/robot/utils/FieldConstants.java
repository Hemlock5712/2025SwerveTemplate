// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.apriltag.AprilTagFields.kDefaultField;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units use explicit unit types
 * and poses have a blue alliance origin.
 */
public class FieldConstants {
  public static final Distance fieldLength = Inches.of(690.876);
  public static final Distance fieldWidth = Inches.of(317);
  public static final Distance startingLineX =
      Inches.of(299.438); // Measured from the inside of starting line
  /**
   * Diameter of the algae game element in meters. Used for game piece manipulation and scoring
   * calculations.
   */
  public static final Distance algaeDiameter = Inches.of(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Inches.of(235.726), Inches.of(0), Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Inches.of(345.428), Inches.of(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Inches.of(345.428), Inches.of(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Inches.of(345.428), Inches.of(199.947));

    // Measured from floor to bottom of cage
    public static final Distance deepHeight = Inches.of(3.125);
    public static final Distance shallowHeight = Inches.of(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(Inches.of(33.526), Inches.of(291.176), Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(Inches.of(33.526), Inches.of(25.824), Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Distance faceLength = Inches.of(36.792600);
    public static final Translation2d center =
        new Translation2d(Inches.of(176.746), Inches.of(158.501));
    public static final Distance faceToZoneLine =
        Inches.of(12); // Side of the reef to the inside of the reef zone
    // line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise
    // order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right
    // branch facing the
    // driver station in
    // clockwise

    private static Translation3d calculateTransform(
        Pose2d poseDirection, Distance adjustX, Distance adjustY, Distance adjustZ) {
      Transform2d adjustment = new Transform2d(adjustX, adjustY, new Rotation2d());
      Pose2d transformed = poseDirection.transformBy(adjustment);
      return new Translation3d(transformed.getMeasureX(), transformed.getMeasureY(), adjustZ);
    }

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(Inches.of(144.003), Inches.of(158.500), Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(Inches.of(160.373), Inches.of(186.857), Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(Inches.of(193.116), Inches.of(186.858), Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(Inches.of(209.489), Inches.of(158.502), Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(Inches.of(193.118), Inches.of(130.145), Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(Inches.of(160.375), Inches.of(130.144), Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          Distance adjustX = Inches.of(30.738);
          Distance adjustY = Inches.of(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  calculateTransform(poseDirection, adjustX, adjustY, level.height),
                  new Rotation3d(
                      Degrees.of(0), level.pitch, poseDirection.getRotation().getMeasure())));
          fillLeft.put(
              level,
              new Pose3d(
                  calculateTransform(poseDirection, adjustX, adjustY.unaryMinus(), level.height),
                  new Rotation3d(
                      Degrees.of(0), level.pitch, poseDirection.getRotation().getMeasure())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Inches.of(48), Inches.of(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Inches.of(48), Inches.of(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Inches.of(48), Inches.of(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Inches.of(72), Degrees.of(-90)),
    L3(Inches.of(47.625), Degrees.of(-35)),
    L2(Inches.of(31.875), Degrees.of(-35)),
    L1(Inches.of(18), Degrees.of(0));

    ReefHeight(Distance height, Angle pitch) {
      this.height = height;
      this.pitch = pitch;
    }

    public final Distance height;
    public final Angle pitch;
  }

  public static final AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(kDefaultField.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
