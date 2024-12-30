// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  /* Hardware */
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, flywheelConfig.reduction()),
          DCMotor.getKrakenX60(1),
          0.001);

  /* Setpoints */
  private double desiredRPM = 0.0;

  /* Control */
  private final PIDController controller = new PIDController(gains.kP(), gains.kI(), gains.kD());

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.flywheelMotorConnected = true;
    inputs.flywheelPositionRads =
        Radians.of(sim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.flywheelVelocityRpm = RPM.of(sim.getAngularVelocityRPM());
    inputs.flywheelAppliedVolts = Volts.of(sim.getInputVoltage());
    inputs.flywheelSupplyCurrentAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.flywheelTorqueCurrentAmps = null;
    inputs.flywheelTempCelsius = null;
    inputs.flywheelSetpointRpm = desiredRPM;
  }

  @Override
  public void runVolts(double flywheelVolts) {
    sim.setInputVoltage(flywheelVolts);
  }

  @Override
  public void stop() {
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setVelocity(double flywheelRpm) {
    desiredRPM = flywheelRpm;

    if (flywheelRpm > 0.0) {
      sim.setAngularVelocity(Units.radiansPerSecondToRotationsPerMinute(flywheelRpm));
    } else {
      sim.setAngularVelocity(0.0);
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void runCharacterizationFlywheel(double input) {
    runVolts(input);
  }
}
