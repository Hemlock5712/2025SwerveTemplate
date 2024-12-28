// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    public SwerveModuleState[] moduleTargets = new SwerveModuleState[4];
    public Pose2d pose = Pose2d.kZero;
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public double odometryPeriod = 0.0;
    public int successfulDaqs = 0;
    public int failedDaqs = 0;

    public AngularVelocity gyroRate = RotationsPerSecond.of(0.0);
    public boolean gyroConnected = false;
    public Rotation2d operatorForwardDirection = new Rotation2d();
    public boolean odometryIsValid = false;

    public Pose2d samplePose = Pose2d.kZero;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  public default void setControl(SwerveRequest request) {}

  public default void seedFieldCentric() {}

  public default void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

  public default void samplePoseAt(DriveIOInputs inputs, double timestampSeconds) {}

  public default void resetPose(Pose2d pose) {}
}
