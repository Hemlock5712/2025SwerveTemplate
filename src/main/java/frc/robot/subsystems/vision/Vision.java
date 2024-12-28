package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionUtil.VisionData;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import frc.robot.subsystems.vision.VisionUtil.VisionMode;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that handles vision processing from multiple cameras using AprilTags. Processes data
 * from MegaTag1 and MegaTag2 vision systems, validates measurements, and provides filtered vision
 * data to the robot's pose estimator.
 */
public class Vision extends SubsystemBase {

  private static final VisionMode MODE = VisionMode.MA;
  private static final String VISION_PATH = "Vision/Camera";

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Creates a new Vision subsystem.
   *
   * @param consumer Callback interface for processed vision measurements
   * @param io Array of VisionIO interfaces for each camera
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");
    this.consumer = consumer;
    this.io = io;

    // Initialize input arrays for each camera
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnection alerts for each camera
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(String.format("Vision camera %d is disconnected.", i), AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    // Update inputs and check connection status for each camera
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
      Logger.processInputs(VISION_PATH + i, inputs[i]);
    }

    // Process vision data and send to consumer
    VisionData visionData = processAllCameras();
    logSummary(visionData);
    consumer.accept(sortMeasurements(visionData.measurements()));
  }

  /**
   * Processes vision data from all cameras and combines the results.
   *
   * @return Combined VisionData from all cameras
   */
  private VisionData processAllCameras() {
    return Arrays.stream(inputs)
        .map(input -> processCamera(Arrays.asList(inputs).indexOf(input), input))
        .reduce(VisionData.empty(), VisionData::merge);
  }

  /**
   * Processes vision data from a single camera.
   *
   * @param cameraIndex Index of the camera being processed
   * @param input Input data from the camera
   * @return Processed VisionData for this camera
   */
  private VisionData processCamera(int cameraIndex, VisionIOInputs input) {
    PoseObservation[] poseObservations = {
      new PoseObservation(input.poseEstimateMT1, input.rawFiducialsMT1),
      new PoseObservation(input.poseEstimateMT2, input.rawFiducialsMT2)
    };

    return Arrays.stream(poseObservations)
        .filter(PoseObservation::isValid)
        .map(observation -> processObservation(cameraIndex, observation))
        .reduce(VisionData.empty(), VisionData::merge);
  }

  /**
   * Processes a single pose observation from a camera.
   *
   * @param cameraIndex Index of the camera that made the observation
   * @param observation The pose observation to process
   * @return Processed VisionData for this observation
   */
  private VisionData processObservation(int cameraIndex, PoseObservation observation) {
    List<VisionMeasurement> measurements = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();
    List<Pose3d> acceptedTagPoses = new ArrayList<>();
    List<Pose3d> rejectedTagPoses = new ArrayList<>();
    List<Pose3d> robotPoses = new ArrayList<>();
    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    Pose3d robotPose = observation.poseEstimate().pose();
    robotPoses.add(robotPose);

    // Validate measurement against current vision mode criteria
    boolean acceptedVisionMeasurement = MODE.acceptVisionMeasurement(observation);

    // Process detected AprilTags
    for (var tag : observation.rawFiducials()) {
      FieldConstants.aprilTags
          .getTagPose(tag.id())
          .ifPresent(
              pose -> {
                tagPoses.add(pose);
                if (acceptedVisionMeasurement) {
                  acceptedTagPoses.add(pose);
                } else {
                  rejectedTagPoses.add(pose);
                }
              });
    }

    // Add to appropriate accepted/rejected lists
    if (acceptedVisionMeasurement) {
      measurements.add(MODE.getVisionMeasurement(observation.poseEstimate()));
      acceptedPoses.add(robotPose);
    } else {
      rejectedPoses.add(robotPose);
    }

    // Create and log vision data
    VisionData data =
        new VisionData(
            measurements,
            tagPoses,
            acceptedTagPoses,
            rejectedTagPoses,
            robotPoses,
            acceptedPoses,
            rejectedPoses);

    String mtType = observation.poseEstimate().isMegaTag2() ? "/MegaTag2" : "/MegaTag1";
    logCameraData(cameraIndex, mtType, data);

    return data;
  }

  /** Logs vision data for a specific camera and MegaTag type. */
  private void logCameraData(int cameraIndex, String mtType, VisionData data) {
    logPoses(VISION_PATH + cameraIndex + mtType, data);
  }

  /** Logs summary of all vision data. */
  private void logSummary(VisionData data) {
    logPoses("Vision/Summary", data);
  }

  /** Logs pose data to AdvantageKit. */
  private void logPoses(String basePath, VisionData data) {
    Logger.recordOutput(basePath + "/TagPoses", toPose3dArray(data.tagPoses()));
    Logger.recordOutput(basePath + "/TagPosesAccepted", toPose3dArray(data.acceptedTagPoses()));
    Logger.recordOutput(basePath + "/TagPosesRejected", toPose3dArray(data.rejectedTagPoses()));
    Logger.recordOutput(basePath + "/RobotPoses", toPose3dArray(data.robotPoses()));
    Logger.recordOutput(basePath + "/RobotPosesAccepted", toPose3dArray(data.acceptedPoses()));
    Logger.recordOutput(basePath + "/RobotPosesRejected", toPose3dArray(data.rejectedPoses()));
  }

  /** Converts a list of poses to an array. */
  private Pose3d[] toPose3dArray(List<Pose3d> poses) {
    return poses.toArray(new Pose3d[poses.size()]);
  }

  /** Sorts vision measurements by timestamp. */
  private List<VisionMeasurement> sortMeasurements(List<VisionMeasurement> measurements) {
    return measurements.stream()
        .sorted(
            (vm1, vm2) ->
                Double.compare(
                    vm1.poseEstimate().timestampSeconds(), vm2.poseEstimate().timestampSeconds()))
        .toList();
  }

  /** Functional interface for consuming processed vision measurements. */
  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(List<VisionMeasurement> visionMeasurements);
  }
}
