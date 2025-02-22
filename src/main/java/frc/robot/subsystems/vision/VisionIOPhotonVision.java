// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  final PhotonCamera camera;
  private final Transform3d robotToCamera;
  final Supplier<VisionParameters> visionParams;
  private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
      TimeInterpolatableBuffer.createBuffer(1.0);

  public VisionIOPhotonVision(
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.visionParams = visionParams;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    DualPoseObservation dualObservation = getEstimatedGlobalPose();
    inputs.poseEstimateMT1 = dualObservation.mt1.poseEstimate();
    inputs.rawFiducialsMT1 = dualObservation.mt1.rawFiducials();
    inputs.poseEstimateMT2 = dualObservation.mt2.poseEstimate();
    inputs.rawFiducialsMT2 = dualObservation.mt2.rawFiducials();
  }

  /**
   * Returns a DualPoseObservation that contains both MT1 and MT2 observations.
   *
   * <ul>
   *   <li>MT1 is computed using either the multitag result or single-target method.
   *   <li>MT2 is computed using the PnP distance-trigonometry strategy but only if exactly one tag
   *       is seen.
   * </ul>
   */
  private DualPoseObservation getEstimatedGlobalPose() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new DualPoseObservation(new PoseObservation(), new PoseObservation());
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return new DualPoseObservation(new PoseObservation(), new PoseObservation());
    }

    // --- Compute MT1 Observation (multi-tag if available, otherwise single-target) ---
    PoseObservation mt1;
    if (latestResult.getMultiTagResult().isPresent()) {
      Transform3d fieldToRobot =
          latestResult.getMultiTagResult().get().estimatedPose.best.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
      // Record heading data.
      addHeadingData(latestResult.getTimestampSeconds(), robotPose.getRotation().toRotation2d());
      mt1 = buildPoseObservation(latestResult, robotPose);
    } else {
      PhotonTrackedTarget target = latestResult.targets.get(0);
      var tagPose = FieldConstants.aprilTags.getTagPose(target.fiducialId);
      if (tagPose.isPresent() && Constants.currentMode != Constants.Mode.SIM) {
        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
        // Record heading data.
        addHeadingData(latestResult.getTimestampSeconds(), robotPose.getRotation().toRotation2d());
        mt1 = buildPoseObservation(latestResult, robotPose);
      } else {
        mt1 = new PoseObservation();
      }
    }

    // --- Compute MT2 Observation (only if exactly one target is seen) ---
    PoseObservation mt2;
    if (latestResult.targets.size() == 1) {
      Optional<EstimatedRobotPose> pnpOpt = pnpDistanceTrigSolveStrategy(latestResult);
      if (pnpOpt.isPresent()) {
        Pose3d robotPose =
            new Pose3d(
                pnpOpt.get().estimatedPose.getTranslation(),
                pnpOpt.get().estimatedPose.getRotation());
        // Record heading data.
        addHeadingData(latestResult.getTimestampSeconds(), robotPose.getRotation().toRotation2d());
        PoseEstimate estimate =
            new PoseEstimate(
                robotPose,
                latestResult.getTimestampSeconds(),
                0.0,
                1, // one tag seen
                0.0,
                robotPose.getTranslation().getNorm(),
                0.0,
                0.0,
                visionParams.get().gyroRate(),
                visionParams.get().robotPose(),
                true);
        RawFiducial[] rawFiducials =
            new RawFiducial[] {createRawFiducial(latestResult.getBestTarget())};
        mt2 = new PoseObservation(estimate, rawFiducials);
      } else {
        mt2 = new PoseObservation();
      }
    } else {
      mt2 = new PoseObservation();
    }

    return new DualPoseObservation(mt1, mt2);
  }

  /** Container class to hold both MT1 and MT2 pose observations. */
  private static class DualPoseObservation {
    final PoseObservation mt1;
    final PoseObservation mt2;

    DualPoseObservation(PoseObservation mt1, PoseObservation mt2) {
      this.mt1 = mt1;
      this.mt2 = mt2;
    }
  }

  private PoseObservation buildPoseObservation(PhotonPipelineResult result, Pose3d robotPose) {
    List<RawFiducial> rawFiducialsList = new ArrayList<>();
    double totalDistance = 0.0;
    double totalArea = 0.0;

    for (var target : result.targets) {
      totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
      totalArea += target.area;
      rawFiducialsList.add(createRawFiducial(target));
    }

    int tagCount = result.targets.size();
    double avgDistance = tagCount > 0 ? totalDistance / tagCount : 0.0;
    double avgArea = tagCount > 0 ? totalArea / tagCount : 0.0;
    double ambiguity = tagCount > 0 ? rawFiducialsList.get(0).ambiguity() : 0.0;

    return new PoseObservation(
        new PoseEstimate(
            robotPose,
            result.getTimestampSeconds(),
            0.0,
            tagCount,
            0.0,
            avgDistance,
            avgArea,
            ambiguity,
            visionParams.get().gyroRate(),
            visionParams.get().robotPose(),
            false),
        rawFiducialsList.toArray(new RawFiducial[0]));
  }

  private RawFiducial createRawFiducial(PhotonTrackedTarget target) {
    return new RawFiducial(
        target.getFiducialId(),
        0,
        0,
        target.area,
        target.bestCameraToTarget.getTranslation().minus(robotToCamera.getTranslation()).getNorm(),
        target.bestCameraToTarget.getTranslation().getNorm(),
        target.poseAmbiguity);
  }

  private Optional<EstimatedRobotPose> pnpDistanceTrigSolveStrategy(PhotonPipelineResult result) {
    PhotonTrackedTarget bestTarget = result.getBestTarget();
    if (bestTarget == null) return Optional.empty();

    Translation2d camToTagTranslation =
        new Pose3d(
                Translation3d.kZero,
                new Rotation3d(
                    0,
                    -Math.toRadians(bestTarget.getPitch()),
                    -Math.toRadians(bestTarget.getYaw())))
            .transformBy(
                new Transform3d(
                    new Translation3d(
                        bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                    Rotation3d.kZero))
            .getTranslation()
            .rotateBy(
                new Rotation3d(
                    robotToCamera.getRotation().getX(), robotToCamera.getRotation().getY(), 0))
            .toTranslation2d();

    if (headingBuffer.getSample(result.getTimestampSeconds()).isEmpty()) {
      return Optional.empty();
    }
    Rotation2d headingSample = headingBuffer.getSample(result.getTimestampSeconds()).get();

    Rotation2d camToTagRotation =
        headingSample.plus(
            robotToCamera.getRotation().toRotation2d().plus(camToTagTranslation.getAngle()));

    if (FieldConstants.aprilTags.getTagPose(bestTarget.getFiducialId()).isEmpty())
      return Optional.empty();
    var tagPose2d =
        FieldConstants.aprilTags.getTagPose(bestTarget.getFiducialId()).get().toPose2d();

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0, Rotation2d.kZero))
            .getTranslation();

    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                headingSample.plus(robotToCamera.getRotation().toRotation2d()))
            .transformBy(
                new Transform2d(
                    new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())
                        .toPose2d(),
                    Pose2d.kZero));

    robotPose = new Pose2d(robotPose.getTranslation(), headingSample);

    return Optional.of(
        new EstimatedRobotPose(
            new Pose3d(robotPose),
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE));
  }

  // Helper method to record heading data into the heading buffer.
  private void addHeadingData(double timestamp, Rotation2d heading) {
    headingBuffer.addSample(timestamp, heading);
  }
}
