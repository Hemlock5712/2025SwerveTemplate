// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for drive subsystem I/O operations. Handles swerve drive state management and pose
 * estimation.
 */
public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    // Module arrays with default states
    public SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    public SwerveModuleState[] moduleTargets =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    public SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    // Position and motion state
    public Pose2d pose = Pose2d.kZero;
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public Rotation2d operatorForwardDirection = Rotation2d.kZero;

    // Diagnostic data
    public double odometryPeriod = 0.0;
    public int successfulDaqs = 0;
    public int failedDaqs = 0;
    public boolean odometryIsValid = false;

    // Sensor data
    public double[] timestamp = new double[0];
    public Rotation2d[] gyroYaw = new Rotation2d[0];
    public AngularVelocity gyroRate = RotationsPerSecond.of(0.0);
    public boolean gyroConnected = false;

    // Module position arrays
    public double[][] drivePositions = new double[4][0];
    public Rotation2d[][] steerPositions = new Rotation2d[4][0];
  }

  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePosition = Radians.of(0.0);
    public AngularVelocity driveVelocity = RotationsPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volt.of(0.0);
    public Current driveStatorCurrent = Amps.of(0.0);
    public Current driveSupplyCurrent = Amps.of(0.0);

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Angle turnAbsolutePosition = Rotations.of(0.0);
    public Angle turnPosition = Rotations.of(0.0);
    public AngularVelocity turnVelocity = RotationsPerSecond.of(0.0);
    public Voltage turnAppliedVolts = Volt.of(0.0);
    public Current turnStatorCurrent = Amps.of(0.0);
    public Current turnSupplyCurrent = Amps.of(0.0);
  }

  default void updateInputs(DriveIOInputs inputs) {}

  default void updateModules(ModuleIOInputs[] inputs) {}

  default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  default void setControl(SwerveRequest request) {}

  default void resetPose(Pose2d pose) {}

  default Optional<Pose2d> samplePoseAt(double timestamp) {
    return Optional.empty();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link #updateWithTime}, then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *     this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
