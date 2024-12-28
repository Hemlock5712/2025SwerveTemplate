package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  private String cameraName;
  private Supplier<VisionParameters> visionParams;

  private final DoubleSubscriber latencySubscriber;

  /**
   * Constructs a Limelight object with the specified camera name and swerve state supplier.
   *
   * @param cameraNames the name of the camera
   * @param swerveStateSupplier the supplier for the swerve drive state
   */
  public VisionIOLimelight(String cameraName, Supplier<VisionParameters> visionParams) {
    this.cameraName = cameraName;
    this.visionParams = visionParams;

    NetworkTable table = LimelightHelpers.getLimelightNTTable(cameraName);
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    VisionParameters currentParams = this.visionParams.get();
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    LimelightHelpers.SetRobotOrientation(
        cameraName,
        currentParams.robotPose().getRotation().getDegrees(),
        currentParams.gyroRate().in(DegreesPerSecond),
        0,
        0,
        0,
        0);

    PoseObservation mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    PoseObservation mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    inputs.poseEstimateMT1 = mt1.poseEstimate().setVisionParams(currentParams);
    inputs.poseEstimateMT2 = mt2.poseEstimate().setVisionParams(currentParams);
    inputs.rawFiducialsMT1 = mt1.rawFiducials();
    inputs.rawFiducialsMT2 = mt2.rawFiducials();
  }
}
