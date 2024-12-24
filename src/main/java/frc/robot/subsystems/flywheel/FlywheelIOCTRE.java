package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOCTRE implements FlywheelIO {
  // Hardware objects
  private final TalonFX flywheelTalon = new TalonFX(10);

  // Inputs from flywheel motor
  private final StatusSignal<Angle> flywheelPosition = flywheelTalon.getPosition();
  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelTalon.getVelocity();
  private final StatusSignal<Voltage> flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
  private final StatusSignal<Current> flywheelCurrent = flywheelTalon.getStatorCurrent();

  // Connection debouncers
  private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5);

  public FlywheelIOCTRE(FlywheelIO io) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);
    flywheelTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Update flywheel inputs
    var flywheelStatus =
        BaseStatusSignal.refreshAll(
            flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    // Update flywheel inputs
    inputs.flywheelConnected = flywheelConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.flywheelPosition = flywheelPosition.getValue();
    inputs.flywheelVelocity = flywheelVelocity.getValue();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
    inputs.flywheelCurrent = flywheelCurrent.getValue();
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    flywheelTalon.stopMotor();
  }
}
