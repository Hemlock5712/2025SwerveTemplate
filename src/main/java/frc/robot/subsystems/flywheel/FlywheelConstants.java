// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.Constants;

public class FlywheelConstants {
  public static final FlywheelConfig flywheelConfig =
      switch (Constants.currentMode) {
        case REAL -> new FlywheelConfig(12, (1.0 / 1.0), 6000.0);
        case SIM -> new FlywheelConfig(0, (1.0 / 1.0), 6000.0);
        case REPLAY -> new FlywheelConfig(12, (1.0 / 1.0), 6000.0);
      };

  public static final Gains gains =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case SIM -> new Gains(1, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
      };

  public static final double FLYWHEEL_SETPOINT_TOLERANCE_RPM = 100;

  public record FlywheelConfig(int motorID, double reduction, double maxAccelerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
