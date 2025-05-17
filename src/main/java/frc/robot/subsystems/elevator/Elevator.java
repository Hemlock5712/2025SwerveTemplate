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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls a dual-motor elevator mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Elevator extends SubsystemBase {
  // Hardware interface and inputs
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  // Current elevator distance mode
  private ElevatorMode currentMode = ElevatorMode.INTAKE;

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
   * @param io The hardware interface implementation for the elevator
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
   * Returns the current distance of the elevator.
   *
   * @return The current angular distance
   */
  @AutoLogOutput
  public Distance getPosition() {
    return inputs.elevatorDistance;
  }

  /** Enumeration of available elevator distances with their corresponding target angles. */
  private enum ElevatorMode {
    STOP(Inches.of(0)), // Stop the elevator
    INTAKE(Inches.of(0)), // Elevator tucked in
    L1(Inches.of(12)), // Position for scoring in L1
    L2(Inches.of(24)), // Position for scoring in L2
    L3(Inches.of(36)), // Position for scoring in L3
    L4(Inches.of(48)); // Position for scoring in L4

    private final Distance targetDistance;
    private final Distance distanceTolerance;

    /**
     * Creates a new elevator mode with a custom distance tolerance.
     *
     * @param targetDistance The target distance for this mode
     * @param distanceTolerance The allowed tolerance from the target distance
     */
    ElevatorMode(Distance targetDistance, Distance distanceTolerance) {
      this.targetDistance = targetDistance;
      this.distanceTolerance = distanceTolerance;
    }

    ElevatorMode(Distance targetDistance) {
      this(targetDistance, Inches.of(2)); // 2 inch default tolerance
    }
  }

  /**
   * Gets the current elevator distance mode.
   *
   * @return The current ElevatorPosition
   */
  public ElevatorMode getMode() {
    return currentMode;
  }

  /**
   * Sets a new elevator distance and schedules the corresponding command.
   *
   * @param mode The desired ElevatorPosition
   */
  private void setElevatorMode(ElevatorMode mode) {
    if (currentMode != mode) {
      currentMode = mode;
      io.setDistance(mode.targetDistance);
    }
  }

  /** Stops the elevator and sets the mode to STOP. */
  private void stopElevator() {
    currentMode = ElevatorMode.STOP;
    io.stop();
  }

  /**
   * Checks if the elevator is at its target distance.
   *
   * @return true if at target distance, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ElevatorMode.STOP) return true;
    return getPosition().isNear(currentMode.targetDistance, currentMode.distanceTolerance);
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
   * Creates a command to set the elevator to a specific distance.
   *
   * @param mode The desired elevator mode
   * @return Command to set the mode
   */
  private Command setPositionCommand(ElevatorMode mode) {
    return Commands.runOnce(() -> setElevatorMode(mode), this)
        .withName("Move to " + mode.toString());
  }

  /**
   * Factory methods for elevator positions
   *
   * <p>Scoring positions: - L1: Low scoring - L2: Mid scoring - L3: High scoring - L4: Extended
   * high scoring
   *
   * <p>Other positions: - intake: Intake position - stop: Emergency stop
   */

  /**
   * @return Command to move the elevator to L1 scoring distance
   */
  public final Command L1() {
    return setPositionCommand(ElevatorMode.L1);
  }

  /**
   * @return Command to move the elevator to L2 scoring distance
   */
  public final Command L2() {
    return setPositionCommand(ElevatorMode.L2);
  }

  /**
   * @return Command to move the elevator to L3 distance
   */
  public final Command L3() {
    return setPositionCommand(ElevatorMode.L3);
  }

  /**
   * @return Command to move the elevator to L4 distance
   */
  public final Command L4() {
    return setPositionCommand(ElevatorMode.L4);
  }

  /**
   * @return Command to intake the elevator
   */
  public final Command intake() {
    return setPositionCommand(ElevatorMode.INTAKE);
  }

  /**
   * @return Command to stop the elevator
   */
  public final Command stop() {
    return Commands.runOnce(this::stopElevator, this).withName("Stop Elevator");
  }
}
