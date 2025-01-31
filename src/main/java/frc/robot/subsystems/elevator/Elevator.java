// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
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
 * The Elevator subsystem controls a dual-motor arm mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Elevator extends SubsystemBase {
  // Hardware interface and inputs
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  // Current arm distance mode
  private ElevatorPosition currentMode = ElevatorPosition.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Elevator leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Elevator follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Elevator encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the arm in closed-loop distance mode to the specified angle.
   *
   * @param distance The target angle distance
   */
  private void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  /** Stops the arm motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current distance of the arm.
   *
   * @return The current angular distance
   */
  @AutoLogOutput
  public Distance getPosition() {
    return inputs.elevatorDistance;
  }

  /** Enumeration of available arm distances with their corresponding target angles. */
  private enum ElevatorPosition {
    STOP(Inches.of(0)), // Stop the arm
    INTAKE(Inches.of(0)), // Elevator tucked in
    L1(Inches.of(12)), // Position for scoring in L1
    L2(Inches.of(24)), // Position for scoring in L2
    L3(Inches.of(36)), // Position for scoring in L3
    L4(Inches.of(48)); // Position for scoring in L4

    private final Distance targetDistance;
    private final Distance angleTolerance;

    ElevatorPosition(Distance targetDistance, Distance angleTolerance) {
      this.targetDistance = targetDistance;
      this.angleTolerance = angleTolerance;
    }

    ElevatorPosition(Distance targetDistance) {
      this(targetDistance, Inches.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current arm distance mode.
   *
   * @return The current ElevatorPosition
   */
  public ElevatorPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm distance and schedules the corresponding command.
   *
   * @param distance The desired ElevatorPosition
   */
  private void setElevatorPosition(ElevatorPosition distance) {
    if (currentMode != distance) {
      if (currentCommand != null) {
        currentCommand.cancel();
      }
      currentMode = distance;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ElevatorPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Elevator"),
              ElevatorPosition.INTAKE,
              createPositionCommand(ElevatorPosition.INTAKE),
              ElevatorPosition.L1,
              createPositionCommand(ElevatorPosition.L1),
              ElevatorPosition.L2,
              createPositionCommand(ElevatorPosition.L2),
              ElevatorPosition.L3,
              createPositionCommand(ElevatorPosition.L3),
              ElevatorPosition.L4,
              createPositionCommand(ElevatorPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific arm distance that moves the arm and checks the target
   * distance.
   *
   * @param distance The arm distance to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ElevatorPosition distance) {
    return Commands.runOnce(() -> setDistance(distance.targetDistance))
        .withName("Move to " + distance.toString());
  }

  /**
   * Checks if the arm is at its target distance.
   *
   * @return true if at target distance, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ElevatorPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetDistance, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Distance targetDistance() {
    return currentMode.targetDistance;
  }

  /**
   * Creates a command to set the arm to a specific distance.
   *
   * @param distance The desired arm distance
   * @return Command to set the distance
   */
  private Command setPositionCommand(ElevatorPosition distance) {
    return Commands.runOnce(() -> setElevatorPosition(distance))
        .withName("SetElevatorPosition(" + distance.toString() + ")");
  }

  /** Factory methods for common distance commands */

  /**
   * @return Command to move the arm to L1 scoring distance
   */
  public final Command L1() {
    return setPositionCommand(ElevatorPosition.L1);
  }

  /**
   * @return Command to move the arm to L2 scoring distance
   */
  public final Command L2() {
    return setPositionCommand(ElevatorPosition.L2);
  }

  /**
   * @return Command to move the arm to L3 distance
   */
  public final Command L3() {
    return setPositionCommand(ElevatorPosition.L3);
  }

  /**
   * @return Command to move the arm to L4 distance
   */
  public final Command L4() {
    return setPositionCommand(ElevatorPosition.L4);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return setPositionCommand(ElevatorPosition.INTAKE);
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ElevatorPosition.STOP);
  }
}
