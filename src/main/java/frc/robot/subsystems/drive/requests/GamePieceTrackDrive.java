package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceTrackDrive extends SwerveSetpointGen {

  private final Translation2d noteLocation = FieldConstants.StagingLocations.spikeTranslations[2];

  public GamePieceTrackDrive(
      ChassisSpeeds currentSpeeds,
      SwerveModuleState[] currentStates,
      Supplier<Pose2d> currentPose) {
    super(currentSpeeds, currentStates, currentPose);
  }

  @Override
  public void applyNative(int id) {
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    double toApplyOmega = RotationalRate;

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
            currentPose.get().getTranslation(),
            noteLocation,
            new Translation2d(toApplyX, toApplyY));

    toApplyX += assistVelocity.getX();
    toApplyY += assistVelocity.getY();

    // Normalize the velocity
    Translation2d normalizedVelocity = normalizeVector(new Translation2d(toApplyX, toApplyY));
    Logger.recordOutput("GamePieceTrackDrive/normalizedVelocity", normalizedVelocity);
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
    Logger.recordOutput("GamePieceTrackDrive/chassisSpeeds", speeds);
    Logger.recordOutput(
        "GamePieceTrackDrive/gamePieceLocation", new Pose2d(noteLocation, new Rotation2d()));

    super.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
        .withVelocityY(speeds.vyMetersPerSecond) // Drive right with X (right)
        .withRotationalRate(speeds.omegaRadiansPerSecond)
        .withOperatorForwardDirection(ForwardDirection);
    super.applyNative(id);
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
    Translation2d robotToNoteVector = notePosition.minus(robotPosition);
    Logger.recordOutput("GamePieceTrackDrive/robotToNoteVector", robotToNoteVector);

    Translation2d perpendicularVector = calculatePerpedicularVector(robotToNoteVector, velocity);
    Logger.recordOutput("GamePieceTrackDrive/perpendicularVector", perpendicularVector);

    Translation2d normalizedVector = normalizeVector(perpendicularVector);
    Logger.recordOutput("GamePieceTrackDrive/normalizedVector", normalizedVector);

    // Scale the assist velocity based on distance to target
    double distance = robotToNoteVector.getNorm();
    // Apply a scaling factor that decreases as we get closer to the target
    double scaleFactor = distance * 0.5; // Add P based on distance

    Translation2d assistVelocity = scaleVectorToMagnitude(normalizedVector, scaleFactor);
    Logger.recordOutput("GamePieceTrackDrive/assistVelocity", assistVelocity);

    return assistVelocity;
  }
}
