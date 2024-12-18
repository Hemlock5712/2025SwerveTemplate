package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;

public class VisionUtil {

  // We are using this to only use MT1 before the match.
  private static boolean BEFORE_MATCH = true;
  public static final double FIELD_MARGIN = 0.5;
  public static final double Z_MARGIN = 0.5;
  public static final double MT2_SPIN_MAX = 40.0;

  // MA Vision Constants
  private static final double MA_VISION_STD_DEV_XY = 0.333;
  private static final double MA_VISION_STD_DEV_THETA = 5;
  public static final double MA_AMBIGUITY = 0.4;

  // PhotonVision Camera Position Constants
  public static final Transform3d CAMERA_LOCATION =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.625), Units.inchesToMeters(11.5), Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(150.)));

  // Enum to represent different modes with unique implementations for each
  public enum VisionMode {
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
    MA {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDev(mt);
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
        if (invalidTime || inValidSpeed || ambiguityThreshold || fieldCheck) {
          return false;
        }
        return true;
      }

      private double calculateXYStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_XY * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }

      private double calculateThetaStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_THETA * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }
    },

    POOF {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDevNew(mt);
        double thetaStdDev = mt.isMegaTag2() ? Double.MAX_VALUE : calculateThetaStdDevNew(mt);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        // final double kMinAreaMegatagEnabled = 0.05;
        // if (poseEstimate.avgTagArea < kMinAreaForMegatag) {
        // Logger.recordOutput(logPreface + "megaTagAvgTagArea", false);
        // return false;
        // }
        // if (poseEstimate.fiducialIds.length != kExpectedTagCount) {
        // Logger.recordOutput(logPreface + "fiducialLength", false);
        // return false;
        // }
        // if (fiducial.ambiguity > .9) {
        // Logger.recordOutput(logPreface + "Ambiguity", false);
        // return false;
        // }
        return false;
      }

      private double calculateXYStdDevNew(PoseEstimate mt) {
        // Placeholder for New Mode XY StdDev calculation
        return Math.log(mt.avgTagDist() + 1) / mt.tagCount();
      }

      private double calculateThetaStdDevNew(PoseEstimate mt) {
        // Placeholder for New Mode Theta StdDev calculation
        return Math.log(mt.avgTagDist() + 1) / mt.tagCount();
      }
    };

    // Abstract methods to be implemented by each mode
    public abstract VisionMeasurement getVisionMeasurement(PoseEstimate mt);

    public abstract boolean acceptVisionMeasurement(PoseObservation mt);
  }

  public record VisionMeasurement(PoseEstimate poseEstimate, Vector<N3> visionMeasurementStdDevs) {}

  public record VisionData(
      List<VisionMeasurement> measurements,
      List<Pose3d> tagPoses,
      List<Pose3d> acceptedTagPoses,
      List<Pose3d> rejectedTagPoses,
      List<Pose3d> robotPoses,
      List<Pose3d> acceptedPoses,
      List<Pose3d> rejectedPoses) {

    // Static factory method for creating empty VisionData
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

    // Helper method to merge two lists
    private static <T> List<T> mergeLists(List<T> list1, List<T> list2) {
      var merged = new ArrayList<T>(list1);
      merged.addAll(list2);
      return merged;
    }

    // Method to combine two VisionData objects
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

  // Method to check if match hasn't started and we should be using MT1 to get
  // starting pose
  private static boolean beforeMatch() {
    return BEFORE_MATCH ? DriverStation.isEnabled() : BEFORE_MATCH;
  }

  private static boolean fieldCheck(Pose3d robotPose) {
    return (robotPose.getX() < -FIELD_MARGIN
        || robotPose.getX() > FieldConstants.fieldLength + FIELD_MARGIN
        || robotPose.getY() < -FIELD_MARGIN
        || robotPose.getY() > FieldConstants.fieldWidth + FIELD_MARGIN
        || robotPose.getZ() < -Z_MARGIN
        || robotPose.getZ() > Z_MARGIN);
  }

  private static boolean speedCheck(double rotationSpeed) {
    return rotationSpeed > MT2_SPIN_MAX;
  }
}
