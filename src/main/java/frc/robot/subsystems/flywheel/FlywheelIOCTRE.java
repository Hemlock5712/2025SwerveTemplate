// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOCTRE implements FlywheelIO {
  // Hardware
  private final TalonFX flywheelTalon;

  /* Setpoints */
  private double desiredRPM = 0.0;

  // Status Signals
  private final StatusSignal<Angle> flywheelPosition;
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelSupplyCurrentAmps;
  private final StatusSignal<Current> flywheelTorqueCurrentAmps;
  private final StatusSignal<Temperature> flywheelTempCelsius;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  // Connection debouncers
  private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5);

  public FlywheelIOCTRE() {
    flywheelTalon = new TalonFX(flywheelConfig.motorID(), "rio");

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = flywheelConfig.reduction();

    // Set signals
    flywheelPosition = flywheelTalon.getPosition();
    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
    flywheelSupplyCurrentAmps = flywheelTalon.getSupplyCurrent();
    flywheelTorqueCurrentAmps = flywheelTalon.getTorqueCurrent();
    flywheelTempCelsius = flywheelTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        flywheelPosition,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelSupplyCurrentAmps,
        flywheelTorqueCurrentAmps,
        flywheelTempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(4.0, flywheelTalon);

    // Apply configs
    flywheelTalon.getConfigurator().apply(config, 1.0);
    flywheelTalon.getConfigurator().apply(controllerConfig, 1.0);

    // Controller config;
    controllerConfig.kP = gains.kP();
    controllerConfig.kI = gains.kI();
    controllerConfig.kD = gains.kD();
    controllerConfig.kS = gains.kS();
    controllerConfig.kV = gains.kV();
    controllerConfig.kA = gains.kA();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Update flywheel inputs
    var flywheelStatus =
        BaseStatusSignal.refreshAll(
            flywheelPosition,
            flywheelVelocity,
            flywheelAppliedVolts,
            flywheelSupplyCurrentAmps,
            flywheelTorqueCurrentAmps,
            flywheelTempCelsius);

    inputs.flywheelMotorConnected = flywheelConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.flywheelPositionRads = flywheelPosition.getValue();
    inputs.flywheelVelocityRpm = flywheelVelocity.getValue();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
    inputs.flywheelSupplyCurrentAmps = flywheelSupplyCurrentAmps.getValue();
    inputs.flywheelTorqueCurrentAmps = flywheelTorqueCurrentAmps.getValue();
    inputs.flywheelTempCelsius = flywheelTempCelsius.getValue();
    inputs.flywheelSetpointRpm = desiredRPM;
  }

  @Override
  public void runVolts(double flywheelVolts) {
    flywheelTalon.setControl(voltageControl.withOutput(flywheelVolts));
  }

  @Override
  public void stop() {
    flywheelTalon.setControl(neutralControl);
  }

  @Override
  public void setVelocity(double flywheelRpm) {
    desiredRPM = flywheelRpm;

    if (flywheelRpm > 0.0) {
      flywheelTalon.setControl(
          velocityControl.withVelocity(flywheelRpm / 60.0).withSlot(0).withEnableFOC(true));
    } else {
      flywheelTalon.setControl(neutralControl);
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    flywheelTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void runCharacterizationFlywheel(double input) {
    flywheelTalon.setControl(voltageControl.withOutput(input));
  }
}
