package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle to
 * ensure the robot is facing the desired direction. Rotation to the target direction is profiled
 * using a trapezoid profile.
 *
 * <p>When users use this request, they specify the direction the robot should travel oriented
 * against the field, and the direction the robot should be facing.
 *
 * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
 * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
 * northward at 5 m/s and turn clockwise to a target of 180 degrees.
 *
 * <p>This control request is especially useful for autonomous control, where the robot should be
 * facing a changing direction throughout the motion.
 */
public class GamePieceTrackDrive implements NativeSwerveRequest {
  /**
   * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   */
  public double VelocityX = 0;
  /**
   * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   */
  public double VelocityY = 0;
  /**
   * The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   */
  public double RotationalRate = 0;

  /** The allowable deadband of the request, in m/s. */
  public double Deadband = 0;
  /** The rotational deadband of the request, in radians per second. */
  public double RotationalDeadband = 0;
  /* The current rotation of the robot */
  public Pose2d CurrentPose = Pose2d.kZero;
  /**
   * The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   */
  public Translation2d CenterOfRotation = new Translation2d();

  /** The type of control request to use for the drive motor. */
  public SwerveModule.DriveRequestType DriveRequestType =
      SwerveModule.DriveRequestType.OpenLoopVoltage;
  /** The type of control request to use for the steer motor. */
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
  /**
   * Whether to desaturate wheel speeds before applying. For more information, see the documentation
   * of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
   */
  public boolean DesaturateWheelSpeeds = false;

  /** The perspective to use when determining which direction is forward. */
  public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

  private final ApplyRobotSpeeds m_swerveSetpoint = new ApplyRobotSpeeds();

  private SwerveSetpoint previousSetpoint;

  private final Translation2d noteLocation = FieldConstants.StagingLocations.spikeTranslations[2];

  /**
   * Creates a new profiled SwerveSetpoint request with the given constraints.
   *
   * @param constraints Constraints for the trapezoid profile
   */
  public GamePieceTrackDrive(ChassisSpeeds currentSpeeds, SwerveModuleState[] currentStates) {
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.PP_CONFIG.numModules));
  }

  public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
    return StatusCode.OK;
  }

  public void applyNative(int id) {
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    double toApplyOmega = RotationalRate;
    Rotation2d toApplyRotation = CurrentPose.getRotation();

    if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      /* If we're operator perspective, modify the X/Y translation by the angle */
      Translation2d tmp = new Translation2d(toApplyX, toApplyY);
      tmp =
          tmp.rotateBy(
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? Rotation2d.kPi
                  : Rotation2d.kZero);
      toApplyX = tmp.getX();
      toApplyY = tmp.getY();
    }

    if (Math.hypot(toApplyX, toApplyY) < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }
    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    Translation2d assistVelocity =
        calculateAssistVelocity(
            CurrentPose.getTranslation(), noteLocation, new Translation2d(toApplyX, toApplyY));

    toApplyX += assistVelocity.getX();
    toApplyY += assistVelocity.getY();

    // Normalize the velocity
    Translation2d normalizedVelocity = normalizeVector(new Translation2d(toApplyX, toApplyY));
    Logger.recordOutput(
        "GamePieceTrackDrive/normalizedVelocity", adjustVectorToRobotPosition(normalizedVelocity));
    double originalMagnitude = Math.hypot(toApplyX, toApplyY);

    Translation2d scaledVelocity = scaleVectorToMagnitude(normalizedVelocity, originalMagnitude);
    toApplyX = scaledVelocity.getX();
    toApplyY = scaledVelocity.getY();
    double magnitude = Math.hypot(toApplyX, toApplyY);
    if (magnitude <= 1e-6) {
      toApplyX = 0.0;
      toApplyY = 0.0;
    }

    ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);
    speeds.toRobotRelativeSpeeds(toApplyRotation);

    Logger.recordOutput("GamePieceTrackDrive/chassisSpeeds", speeds);
    Logger.recordOutput(
        "GamePieceTrackDrive/gamePieceLocation", new Pose2d(noteLocation, new Rotation2d()));

    previousSetpoint =
        Constants.setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );

    m_swerveSetpoint
        .withSpeeds(previousSetpoint.robotRelativeSpeeds())
        // .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesX())
        // .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesY())
        .withCenterOfRotation(CenterOfRotation)
        .withDriveRequestType(DriveRequestType)
        .withSteerRequestType(SteerRequestType)
        .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
        .applyNative(id);
  }

  private Translation2d adjustVectorToRobotPosition(Translation2d vector) {
    return CurrentPose.getTranslation().minus(vector);
  }

  private Translation2d vectorToTranslation(Vector<N2> vector) {
    return new Translation2d(vector.get(0), vector.get(1));
  }

  private Translation2d calculatePerpedicularVector(
      Translation2d robotToNote, Translation2d velocity) {
    Vector<N2> robotToNoteVector = robotToNote.toVector();
    Vector<N2> velocityVector = velocity.toVector();
    double velocityProj = velocityVector.dot(velocityVector);
    if (velocityProj == 0) {
      return new Translation2d();
    }
    double projectionFactor = robotToNoteVector.dot(velocityVector) / velocityProj;
    Vector<N2> projectionVector = velocityVector.times(projectionFactor);

    Vector<N2> perpendicularVector = robotToNoteVector.minus(projectionVector);

    return vectorToTranslation(perpendicularVector);
  }

  private Translation2d normalizeVector(Translation2d vector) {
    double normalized = vector.getNorm();

    if (normalized == 0) {
      return vector;
    }

    return vector.div(normalized);
  }

  private Translation2d scaleVectorToMagnitude(Translation2d vector, double magnitude) {
    Translation2d normalizedVector = normalizeVector(vector);
    return normalizedVector.times(magnitude);
  }

  private Translation2d calculateAssistVelocity(
      Translation2d robotPosition, Translation2d notePosition, Translation2d velocity) {
    Translation2d robotToNoteVector = robotPosition.minus(notePosition);
    Logger.recordOutput(
        "GamePieceTrackDrive/robotToNoteVector", adjustVectorToRobotPosition(robotToNoteVector));

    Translation2d perpendicularVector = calculatePerpedicularVector(robotToNoteVector, velocity);
    Logger.recordOutput(
        "GamePieceTrackDrive/perpendicularVector",
        adjustVectorToRobotPosition(perpendicularVector));

    Translation2d normalizedVector = normalizeVector(perpendicularVector);
    Logger.recordOutput(
        "GamePieceTrackDrive/normalizedVector", adjustVectorToRobotPosition(normalizedVector));

    // Scale the assist velocity based on distance to target
    double distance = robotToNoteVector.getNorm();
    // Apply a scaling factor that decreases as we get closer to the target
    double scaleFactor = Math.min(0.3, distance * 0.1); // Max 0.3, proportional to distance

    Translation2d assistVelocity = scaleVectorToMagnitude(normalizedVector, scaleFactor);
    Logger.recordOutput(
        "GamePieceTrackDrive/assistVelocity", adjustVectorToRobotPosition(assistVelocity));

    return assistVelocity;
  }

  /**
   * Modifies the VelocityX parameter and returns itself.
   *
   * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param newVelocityX Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withVelocityX(double newVelocityX) {
    this.VelocityX = newVelocityX;
    return this;
  }

  /**
   * Modifies the VelocityX parameter and returns itself.
   *
   * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param newVelocityX Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withVelocityX(LinearVelocity newVelocityX) {
    this.VelocityX = newVelocityX.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the VelocityY parameter and returns itself.
   *
   * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param newVelocityY Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withVelocityY(double newVelocityY) {
    this.VelocityY = newVelocityY;
    return this;
  }

  /**
   * Modifies the VelocityY parameter and returns itself.
   *
   * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param newVelocityY Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withVelocityY(LinearVelocity newVelocityY) {
    this.VelocityY = newVelocityY.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the RotationalRate parameter and returns itself.
   *
   * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   *
   * @param newRotationalRate Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withRotationalRate(double newRotationalRate) {
    this.RotationalRate = newRotationalRate;
    return this;
  }

  /**
   * Modifies the RotationalRate parameter and returns itself.
   *
   * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   *
   * @param newRotationalRate Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withRotationalRate(AngularVelocity newRotationalRate) {
    this.RotationalRate = newRotationalRate.in(RadiansPerSecond);
    return this;
  }

  /**
   * Modifies the Deadband parameter and returns itself.
   *
   * <p>The allowable deadband of the request, in m/s.
   *
   * @param newDeadband Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withDeadband(double newDeadband) {
    this.Deadband = newDeadband;
    return this;
  }

  /**
   * Modifies the Deadband parameter and returns itself.
   *
   * <p>The allowable deadband of the request, in m/s.
   *
   * @param newDeadband Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withDeadband(LinearVelocity newDeadband) {
    this.Deadband = newDeadband.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the RotationalDeadband parameter and returns itself.
   *
   * <p>The rotational deadband of the request, in radians per second.
   *
   * @param newRotationalDeadband Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withRotationalDeadband(double newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband;
    return this;
  }

  /**
   * Modifies the RotationalDeadband parameter and returns itself.
   *
   * <p>The rotational deadband of the request, in radians per second.
   *
   * @param newRotationalDeadband Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withRotationalDeadband(AngularVelocity newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
    return this;
  }

  /**
   * Modifies the current Pose.
   *
   * <p>The current pose in Pose2d
   *
   * @param newPose Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withPose(Pose2d newPose) {
    this.CurrentPose = newPose;
    return this;
  }

  /**
   * Modifies the CenterOfRotation parameter and returns itself.
   *
   * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   *
   * @param newCenterOfRotation Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withCenterOfRotation(Translation2d newCenterOfRotation) {
    this.CenterOfRotation = newCenterOfRotation;
    return this;
  }

  /**
   * Modifies the DriveRequestType parameter and returns itself.
   *
   * <p>The type of control request to use for the drive motor.
   *
   * @param newDriveRequestType Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withDriveRequestType(
      SwerveModule.DriveRequestType newDriveRequestType) {
    this.DriveRequestType = newDriveRequestType;
    return this;
  }

  /**
   * Modifies the SteerRequestType parameter and returns itself.
   *
   * <p>The type of control request to use for the drive motor.
   *
   * @param newSteerRequestType Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withSteerRequestType(
      SwerveModule.SteerRequestType newSteerRequestType) {
    this.SteerRequestType = newSteerRequestType;
    return this;
  }

  /**
   * Modifies the DesaturateWheelSpeeds parameter and returns itself.
   *
   * <p>Whether to desaturate wheel speeds before applying. For more information, see the
   * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
   *
   * @param newDesaturateWheelSpeeds Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
    this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
    return this;
  }

  /**
   * Modifies the ForwardPerspective parameter and returns itself.
   *
   * <p>The perspective to use when determining which direction is forward.
   *
   * @param newForwardPerspective Parameter to modify
   * @return this object
   */
  public GamePieceTrackDrive withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
    this.ForwardPerspective = newForwardPerspective;
    return this;
  }
}
