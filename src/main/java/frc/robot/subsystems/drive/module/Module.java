// Copyright 2021-2024 FRC 5712
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.DebouncedAlert;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final DebouncedAlert driveDisconnectedAlert;
  private final DebouncedAlert turnDisconnectedAlert;
  private final DebouncedAlert turnEncoderDisconnectedAlert;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new DebouncedAlert(
            "Drive motor disconnected or booting on module " + Integer.toString(index),
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            0.5);
    turnDisconnectedAlert =
        new DebouncedAlert(
            "Turn motor disconnected or booting on module " + Integer.toString(index),
            "Disconnected turn motor on module " + Integer.toString(index) + ".",
            0.5);
    turnEncoderDisconnectedAlert =
        new DebouncedAlert(
            "Turn encoder disconnected or booting on module " + Integer.toString(index),
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            0.5);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  public Angle getDrivePosition() {
    return inputs.drivePosition;
  }
}
