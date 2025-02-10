// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Flywheel subsystem controls a dual-motor flywheel mechanism used for shooting game pieces. It
 * supports multiple shooting modes with different target speeds and provides closed-loop control.
 */
public class Flywheel extends SubsystemBase {
  // Hardware interface and inputs
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  // Current flywheel mode
  private FlywheelMode currentMode = FlywheelMode.STOP;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Flywheel leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Flywheel follower motor isn't connected", AlertType.kError);

  /**
   * Creates a new Flywheel subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the flywheel
   */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  /**
   * Runs the flywheel in closed-loop velocity mode at the specified speed.
   *
   * @param velocity The target angular velocity
   */
  private void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the flywheel motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current velocity of the flywheel.
   *
   * @return The current angular velocity
   */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.leaderVelocity;
  }

  /** Enumeration of available flywheel modes with their corresponding target speeds. */
  private enum FlywheelMode {
    STOP(RotationsPerSecond.of(0)), // Stop the flywheel
    L1(RotationsPerSecond.of(5)), // For scoring at L1
    L2(RotationsPerSecond.of(10)), // For scoring at L2
    L3(RotationsPerSecond.of(20)); // For scoring at L3

    private final AngularVelocity targetSpeed;
    private final AngularVelocity speedTolerance;

    FlywheelMode(AngularVelocity targetSpeed, AngularVelocity speedTolerance) {
      this.targetSpeed = targetSpeed;
      this.speedTolerance = speedTolerance;
    }

    FlywheelMode(AngularVelocity targetSpeed) {
      this(targetSpeed, RotationsPerSecond.of(1)); // 1 RPS default tolerance
    }
  }

  /**
   * Gets the current flywheel mode.
   *
   * @return The current Flywheel mode
   */
  public FlywheelMode getMode() {
    return currentMode;
  }

  // Command that runs the appropriate routine based on the current mode
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              FlywheelMode.STOP,
              Commands.runOnce(this::stop).withName("Stop Flywheel"),
              FlywheelMode.L1,
              createVelocityCommand(FlywheelMode.L1),
              FlywheelMode.L2,
              createVelocityCommand(FlywheelMode.L2),
              FlywheelMode.L3,
              createVelocityCommand(FlywheelMode.L3)),
          this::getMode);

  /**
   * Sets a new flywheel mode and schedules the corresponding command.
   *
   * @param mode The desired FlywheelMode
   */
  private void setFlywheelMode(FlywheelMode mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  /**
   * Creates a command for a specific flywheel mode that runs the flywheel and checks the target
   * speed.
   *
   * @param mode The flywheel mode to create a command for
   * @return A command that implements the flywheel movement
   */
  private Command createVelocityCommand(FlywheelMode mode) {
    return Commands.runOnce(() -> setVelocity(mode.targetSpeed)).withName("Run " + mode.toString());
  }

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == FlywheelMode.STOP) return true;
    return getVelocity().isNear(currentMode.targetSpeed, currentMode.speedTolerance);
  }

  /**
   * Logs target speed for given mode.
   *
   * @return The target speed for the current mode
   */
  @AutoLogOutput
  private AngularVelocity targetSpeed() {
    return currentMode.targetSpeed;
  }

  /**
   * Creates a command to set the flywheel to a specific mode.
   *
   * @param mode The desired flywheel mode
   * @return Command to set the mode
   */
  private Command setVelocityCommand(FlywheelMode mode) {
    return Commands.runOnce(() -> setFlywheelMode(mode))
        .withName("SetFlywheelMode(" + mode.toString() + ")");
  }

  /** Factory methods for common mode commands */

  /**
   * @return Command to set the flywheel to L1 mode
   */
  public final Command L1() {
    return setVelocityCommand(FlywheelMode.L1);
  }

  /**
   * @return Command to set the flywheel to L2 mode
   */
  public final Command L2() {
    return setVelocityCommand(FlywheelMode.L2);
  }

  /**
   * @return Command to set the flywheel to L3 mode
   */
  public final Command L3() {
    return setVelocityCommand(FlywheelMode.L3);
  }

  /**
   * @return Command to stop the flywheel
   */
  public final Command stopCommand() {
    return setVelocityCommand(FlywheelMode.STOP);
  }
}
