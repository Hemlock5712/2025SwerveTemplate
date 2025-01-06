// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  final PhotonCamera camera;
  private final Transform3d robotToCamera;
  final Supplier<VisionParameters> visionParams;

  public VisionIOPhotonVision(
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.visionParams = visionParams;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    PoseObservation observation = getEstimatedGlobalPose();
    inputs.poseEstimateMT1 = observation.poseEstimate();
    inputs.rawFiducialsMT1 = observation.rawFiducials();
  }

  private PoseObservation getEstimatedGlobalPose() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) return new PoseObservation();

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return new PoseObservation();
    }
    var multitagResult = latestResult.getMultiTagResult();
    if (multitagResult.isPresent()) {
      Transform3d fieldToRobot =
          multitagResult.get().estimatedPose.best.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
      return buildPoseObservation(latestResult, robotPose);
    }
    var target = latestResult.targets.get(0);
    // Calculate robot pose
    var tagPose = FieldConstants.aprilTags.getTagPose(target.fiducialId);
    if (tagPose.isPresent() && Constants.currentMode != Constants.Mode.SIM) {
      Transform3d fieldToTarget =
          new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
      Transform3d cameraToTarget = target.bestCameraToTarget;
      Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
      Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
      return buildPoseObservation(latestResult, robotPose);
    }
    return new PoseObservation();
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
            visionParams.get().gyroRate().in(DegreesPerSecond),
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
}
