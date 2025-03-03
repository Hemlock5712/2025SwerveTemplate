// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.Robot;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for vision processing that provides different vision modes, measurement validation,
 * and data structures for vision measurements. Supports both MegaTag1 (MT1) and MegaTag2 (MT2)
 * vision systems.
 */
public class VisionUtil {
  public static final Distance FIELD_MARGIN =
      Meters.of(0.5); // Meters beyond field boundaries to accept measurements
  public static final Distance Z_MARGIN =
      Meters.of(0.5); // Meters above/below field to accept measurements
  public static final AngularVelocity MT2_SPIN_MAX =
      DegreesPerSecond.of(40.0); // Maximum rotation speed for MT2 measurements
  public static final double MIN_TAG_AREA = 0.05; // Minimum tag area to be accepted

  // Vision measurement constants for MA mode
  private static final double MA_VISION_STD_DEV_XY = 0.333; // Base XY standard deviation
  private static final double MA_VISION_STD_DEV_THETA = 5.0; // Base theta standard deviation
  public static final double MA_AMBIGUITY =
      0.4; // Maximum allowed ambiguity for single-tag measurements

  /**
   * Enum defining different vision processing modes with unique validation and measurement
   * calculation implementations.
   */
  public enum VisionMode {
    /**
     * Mode that rejects all vision measurements. Useful for testing or when vision should be
     * temporarily disabled.
     */
    NONE {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        return new VisionMeasurement(
            mt, VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return false;
      }
    },

    /**
     * Standard vision mode that calculates measurement uncertainties based on distance. Uses a
     * simple model where standard deviations increase quadratically with distance and decrease
     * linearly with the number of tags detected.
     */
    MA {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateStdDev(mt, MA_VISION_STD_DEV_XY);
        // MT2 measurements don't provide reliable rotation data
        double thetaStdDev =
            mt.isMegaTag2() ? Double.MAX_VALUE : calculateStdDev(mt, MA_VISION_STD_DEV_THETA);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      /**
       * Calculates the standard deviation for X and Y measurements.
       *
       * @param mt The pose estimate containing tag detection information
       * @param scaler Amount to scale trust by. Smaller is greater trust
       * @return Standard deviation scaled by distance squared and tag count
       */
      private double calculateStdDev(PoseEstimate mt, double scaler) {
        return scaler * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }
    },

    /**
     * Implements generic version POOF (Cheesy Poof 254) algorithm. These values where chosen due to
     * how the distance should be similar from last game to this game (being feed vs cycling). You
     * will want to tune.
     */
    POOF {

      /**
       * Calculates vision measurements using the POOF algorithm.
       *
       * @param mt The pose estimate to process
       * @return A VisionMeasurement with calculated standard deviations
       */
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        var stdDevs = calculateStandardDeviations(mt);
        var xyStdDev = stdDevs.xyStdDev();
        var thetaStdDev = stdDevs.thetaStdDev();

        // MT2 measurements don't provide reliable rotation data
        if (mt.isMegaTag2()) {
          thetaStdDev = Double.MAX_VALUE;
        }

        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      /** Custom implementation that take default behavior and add area filter */
      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return super.acceptVisionMeasurement(mt) && !invalidTagArea(mt.poseEstimate());
      }
    };

    /**
     * Record holding vision standard deviation values for both position and rotation.
     *
     * @param xyStdDev Standard deviation for X and Y measurements
     * @param thetaStdDev Standard deviation for rotation measurements
     */
    private record VisionDevs(double xyStdDev, double thetaStdDev) {}

    /**
     * Calculates standard deviations based on the number of tags detected.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs containing calculated standard deviations
     */
    private static VisionDevs calculateStandardDeviations(PoseEstimate mt) {
      if (mt.tagCount() >= 2) {
        return calculateMultipleTagStdDevs(mt);
      } else if (mt.tagCount() == 1) {
        return calculateSingleTagStdDevs(mt);
      }
      // No tags detected - return maximum uncertainty
      return new VisionDevs(Double.MAX_VALUE, Double.MAX_VALUE);
    }

    /**
     * Calculates standard deviations when multiple tags are detected. Uses tag area to determine
     * measurement confidence.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs with appropriate standard deviations
     */
    private static VisionDevs calculateMultipleTagStdDevs(PoseEstimate mt) {
      boolean hasLargeTagArea = mt.avgTagArea() > 0.1;
      // Higher confidence (smaller std devs) when tags appear larger in the image
      return hasLargeTagArea
          ? new VisionDevs(0.2, Units.degreesToRadians(6.0))
          : new VisionDevs(1.2, Units.degreesToRadians(12.0));
    }

    /**
     * Calculates standard deviations when only one tag is detected. Uses both tag area and distance
     * from expected pose to determine confidence.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs with appropriate standard deviations
     */
    private static VisionDevs calculateSingleTagStdDevs(PoseEstimate mt) {
      // Calculate how far the measured pose is from the expected pose
      double poseDifference =
          mt.robotPose().getTranslation().getDistance(mt.pose().getTranslation().toTranslation2d());

      // Define confidence thresholds based on pose difference and tag area
      boolean isCloseToExpectedPose = poseDifference < 0.5;
      boolean isVeryCloseToExpectedPose = poseDifference < 0.3;
      boolean hasLargeTagArea = mt.avgTagArea() > 0.1;

      // Assign standard deviations based on confidence levels
      if (mt.avgTagArea() > 0.8 && isCloseToExpectedPose) {
        // Highest confidence case
        return new VisionDevs(0.5, Units.degreesToRadians(12.0));
      } else if (hasLargeTagArea && isVeryCloseToExpectedPose) {
        // Medium confidence case
        return new VisionDevs(1.0, Units.degreesToRadians(25.0));
      }
      // Low confidence case
      return new VisionDevs(2.0, Units.degreesToRadians(50.0));
    }

    /**
     * Creates a vision measurement with calculated standard deviations.
     *
     * @param mt The pose estimate to process
     * @return A vision measurement with appropriate standard deviations
     */
    public abstract VisionMeasurement getVisionMeasurement(PoseEstimate mt);

    /**
     * Default implementation for validating vision measurements. Checks all standard validation
     * criteria. This implementation is shared between MA and POOF modes.
     *
     * @param mt The pose observation to validate
     * @return True if the measurement should be accepted, false otherwise
     */
    public boolean acceptVisionMeasurement(PoseObservation mt) {
      if (!mt.isValid()) {
        return false;
      }
      PoseEstimate poseEst = mt.poseEstimate();
      return !invalidPose(poseEst.pose())
          && !invalidMT2Time(poseEst)
          && !invalidRotationVelocity(poseEst)
          && !invalidAmbiguity(poseEst);
    }
  }

  /** Record containing a pose estimate and its associated standard deviations. */
  public record VisionMeasurement(PoseEstimate poseEstimate, Vector<N3> visionMeasurementStdDevs) {}

  /**
   * Record containing collections of vision measurements and poses, categorized by acceptance
   * status.
   */
  public record VisionData(
      List<VisionMeasurement> measurements,
      List<Pose3d> tagPoses,
      List<Pose3d> acceptedTagPoses,
      List<Pose3d> rejectedTagPoses,
      List<Pose3d> robotPoses,
      List<Pose3d> acceptedPoses,
      List<Pose3d> rejectedPoses) {

    /** Creates an empty VisionData object. */
    public static VisionData empty() {
      return new VisionData(
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>());
    }

    /** Merges two lists of the same type. */
    private static <T> List<T> mergeLists(List<T> list1, List<T> list2) {
      ArrayList<T> merged = new ArrayList<>(list1);
      merged.addAll(list2);
      return merged;
    }

    /**
     * Combines this VisionData with another, merging all corresponding lists.
     *
     * @param other The VisionData to merge with
     * @return A new VisionData containing all elements from both objects
     */
    public VisionData merge(VisionData other) {
      return new VisionData(
          mergeLists(measurements, other.measurements),
          mergeLists(tagPoses, other.tagPoses),
          mergeLists(acceptedTagPoses, other.acceptedTagPoses),
          mergeLists(rejectedTagPoses, other.rejectedTagPoses),
          mergeLists(robotPoses, other.robotPoses),
          mergeLists(acceptedPoses, other.acceptedPoses),
          mergeLists(rejectedPoses, other.rejectedPoses));
    }
  }

  /**
   * Validation helper method to check if MT2 measurements are being used before match start.
   *
   * @param mt The pose estimate to validate
   * @return True if the measurement is invalid due to timing constraints
   */
  private static boolean invalidMT2Time(PoseEstimate mt) {
    return mt.isMegaTag2() && Robot.BEFORE_MATCH;
  }

  /**
   * Validation helper method to check if rotation velocity is within acceptable limits.
   *
   * @param mt The pose estimate to validate
   * @return True if the rotation velocity exceeds the maximum allowed speed
   */
  private static boolean invalidRotationVelocity(PoseEstimate mt) {
    return Math.abs(mt.yawVelocity()) > MT2_SPIN_MAX.in(DegreesPerSecond);
  }

  /**
   * Validation helper method to check if ambiguity is acceptable for single-tag measurements.
   *
   * @param mt The pose estimate to validate
   * @return True if the ambiguity is too high for a single-tag measurement
   */
  private static boolean invalidAmbiguity(PoseEstimate mt) {
    return mt.tagCount() == 1 && mt.ambiguity() > MA_AMBIGUITY;
  }

  /**
   * Validation helper method to check if the pose is within the valid field boundaries.
   *
   * @param robotPose The 3D pose to validate
   * @return True if the pose is outside the valid field boundaries
   */
  private static boolean invalidPose(Pose3d robotPose) {
    return robotPose.getMeasureX().lt(FIELD_MARGIN.unaryMinus())
        || robotPose.getMeasureX().gt(FieldConstants.fieldLength.plus(FIELD_MARGIN))
        || robotPose.getMeasureY().lt(FIELD_MARGIN.unaryMinus())
        || robotPose.getMeasureY().gt(FieldConstants.fieldWidth.plus(FIELD_MARGIN))
        || robotPose.getMeasureZ().lt(Z_MARGIN.unaryMinus())
        || robotPose.getMeasureZ().gt(Z_MARGIN);
  }

  /**
   * Validation helper method to check if the average tag area is sufficient.
   *
   * @param mt The pose estimate to validate
   * @return True if the average tag area is below the minimum threshold
   */
  private static boolean invalidTagArea(PoseEstimate mt) {
    return mt.avgTagArea() < MIN_TAG_AREA;
  }
}
