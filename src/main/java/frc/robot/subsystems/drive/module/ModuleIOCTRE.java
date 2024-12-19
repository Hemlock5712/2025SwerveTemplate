package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ModuleIOCTRE implements ModuleIO {
  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnStatorCurrent;
  private final StatusSignal<Current> turnTorqueCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOCTRE(SwerveModule module) {
    // Get hardware objects
    driveTalon = module.getDriveMotor();
    turnTalon = module.getSteerMotor();
    cancoder = module.getCANcoder();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnStatorCurrent = turnTalon.getStatorCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveStatorCurrent,
        driveTorqueCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnTorqueCurrent);
    ParentDevice.optimizeBusUtilizationForAll(4.0, driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveStatorCurrent,
            driveTorqueCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition, turnVelocity, turnAppliedVolts, turnStatorCurrent, turnTorqueCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePosition = drivePosition.getValue();
    inputs.driveVelocity = driveVelocity.getValue();
    inputs.driveAppliedVolts = driveAppliedVolts.getValue();
    inputs.driveStatorCurrent = driveStatorCurrent.getValue();
    inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocity = turnVelocity.getValue();
    inputs.turnAppliedVolts = turnAppliedVolts.getValue();
    inputs.turnStatorCurrent = turnStatorCurrent.getValue();
    inputs.turnTorqueCurrent = turnTorqueCurrent.getValue();
  }
}
