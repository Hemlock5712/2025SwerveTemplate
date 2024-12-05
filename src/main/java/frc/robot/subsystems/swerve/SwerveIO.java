// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    public SwerveModuleState[] moduleTargets = new SwerveModuleState[4];
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public double odometryPeriod;
    public int successfulDaqs;
    public int failedDaqs;

    public double gyroRate;
    public Rotation3d rotation3d = new Rotation3d();
    public boolean odometryIsValid;
  }

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  public default void setControl(SwerveRequest request) {}

  public default void seedFieldCentric() {}

  public default void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

  public default void resetPose(Pose2d pose) {}
}