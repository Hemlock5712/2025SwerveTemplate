package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for vision processing that provides different vision modes, measurement validation,
 * and data structures for vision measurements. Supports both MegaTag1 (MT1) and MegaTag2 (MT2)
 * vision systems.
 */
public class VisionUtil {
  // Configuration constants
  private static boolean BEFORE_MATCH = true; // Controls MT1-only usage before match
  public static final double FIELD_MARGIN =
      0.5; // Meters beyond field boundaries to accept measurements
  public static final double Z_MARGIN = 0.5; // Meters above/below field to accept measurements
  public static final double MT2_SPIN_MAX = 40.0; // Maximum rotation speed for MT2 measurements

  // Vision measurement constants for MA mode
  private static final double MA_VISION_STD_DEV_XY = 0.333;
  private static final double MA_VISION_STD_DEV_THETA = 5;
  public static final double MA_AMBIGUITY = 0.4;

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
        return new VisionMeasurement(mt, VecBuilder.fill(0.0, 0.0, 0.0));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return false;
      }
    },

    /** Standard vision mode with distance-based standard deviations and basic validation checks. */
    MA {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDev(mt);
        // MT2 measurements don't provide reliable rotation data
        double thetaStdDev = mt.isMegaTag2() ? Double.MAX_VALUE : calculateThetaStdDev(mt);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        boolean invalidTime = mt.poseEstimate().isMegaTag2() && beforeMatch();
        boolean ambiguityThreshold =
            mt.poseEstimate().tagCount() == 1 && mt.poseEstimate().ambiguity() > MA_AMBIGUITY;
        boolean fieldCheck = fieldCheck(mt.poseEstimate().pose());
        boolean inValidSpeed = speedCheck(mt.poseEstimate().yawVelocityRadPerSec());
        // Reject if any validation checks fail
        return invalidTime || inValidSpeed || ambiguityThreshold || fieldCheck;
      }

      private double calculateXYStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_XY * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }

      private double calculateThetaStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_THETA * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }
    },

    /**
     * Experimental vision mode with alternative validation checks and measurement calculations.
     * Currently disabled.
     */
    POOF {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDevNew(mt);
        double thetaStdDev = mt.isMegaTag2() ? Double.MAX_VALUE : calculateThetaStdDevNew(mt);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        // TODO: Implement validation checks for POOF mode
        return false;
      }

      private double calculateXYStdDevNew(PoseEstimate mt) {
        // TODO: Implement POOF logic for XY standard deviation
        return Math.log(mt.avgTagDist() + 1) / mt.tagCount();
      }

      private double calculateThetaStdDevNew(PoseEstimate mt) {
        // TODO: Implement POOF logic for theta standard deviation
        return Math.log(mt.avgTagDist() + 1) / mt.tagCount();
      }
    };

    /**
     * Creates a vision measurement with calculated standard deviations.
     *
     * @param mt The pose estimate to process
     * @return A vision measurement with appropriate standard deviations
     */
    public abstract VisionMeasurement getVisionMeasurement(PoseEstimate mt);

    /**
     * Determines whether a vision measurement should be accepted.
     *
     * @param mt The pose observation to validate
     * @return True if the measurement should be accepted
     */
    public abstract boolean acceptVisionMeasurement(PoseObservation mt);
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

  /** Checks if the robot is in the pre-match phase where only MT1 should be used. */
  private static boolean beforeMatch() {
    if (DriverStation.isEnabled()) {
      BEFORE_MATCH = false;
    }
    return BEFORE_MATCH && !DriverStation.isEnabled();
  }

  /** Validates that a pose is within the field boundaries (plus margins). */
  private static boolean fieldCheck(Pose3d robotPose) {
    return robotPose.getX() < -FIELD_MARGIN
        || robotPose.getX() > FieldConstants.fieldLength + FIELD_MARGIN
        || robotPose.getY() < -FIELD_MARGIN
        || robotPose.getY() > FieldConstants.fieldWidth + FIELD_MARGIN
        || robotPose.getZ() < -Z_MARGIN
        || robotPose.getZ() > Z_MARGIN;
  }

  /** Validates that the robot's rotation speed is within acceptable limits. */
  private static boolean speedCheck(double rotationSpeed) {
    return Math.abs(rotationSpeed) > MT2_SPIN_MAX;
  }
}
