package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /* Disconnected alerts */
  private final Alert flywheelDisconnectedAlert;

  /* System variables and parameters */
  private FlywheelState systemState = FlywheelState.IDLE;

  private boolean requestLow = false;
  private boolean requestMedium = false;
  private boolean requestHigh = false;

  private double stateStartTime = 0.0;

  /* Setpoints */
  private double desiredRPM = 0.0;

  /* System states */
  public enum FlywheelState {
    IDLE,
    LOW,
    MEDIUM,
    HIGH
  }

  public Flywheel(FlywheelIO io) {
    System.out.println("[Init] Creating Flywheel");
    this.io = io;
    flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/SystemState", systemState);

    /* Update alerts */
    flywheelDisconnectedAlert.set(!inputs.flywheelMotorConnected);

    /* Handle statemachine logic */
    FlywheelState nextSystemState = systemState;

    if (systemState == FlywheelState.IDLE) {
      io.setVelocity(0);

      if (requestLow) {
        nextSystemState = FlywheelState.LOW;
      } else if (requestMedium) {
        nextSystemState = FlywheelState.MEDIUM;
      } else if (requestHigh) {
        nextSystemState = FlywheelState.HIGH;
      }
    } else if (systemState == FlywheelState.LOW) {
      io.setVelocity(1000);

      if (!requestLow) {
        nextSystemState = FlywheelState.IDLE;
      }
    } else if (systemState == FlywheelState.MEDIUM) {
      io.setVelocity(2000);

      if (!requestMedium) {
        nextSystemState = FlywheelState.IDLE;
      }
    } else if (systemState == FlywheelState.HIGH) {
      io.setVelocity(3000);

      if (!requestHigh) {
        nextSystemState = FlywheelState.IDLE;
      }
    }
    if (nextSystemState != systemState) {
      stateStartTime = RobotController.getFPGATime() / 1.0E6;
      systemState = nextSystemState;
    }
  }
  // Returns whether the shooter is at setpoint for the superstructure
  public boolean atSetpoint() {
    Logger.recordOutput("Flywheel/RPM error", desiredRPM - inputs.flywheelVelocityRpm.abs(RPM));
    return Math.abs(desiredRPM - inputs.flywheelVelocityRpm.abs(RPM))
        < FLYWHEEL_SETPOINT_TOLERANCE_RPM;
  }

  public void requestIdle() {
    desiredRPM = 0;
    unsetAllRequests();
  }

  public void requestLow() {
    desiredRPM = 2000;
    requestLow = true;
  }

  public void requestMedium() {
    desiredRPM = 4000;
    requestMedium = true;
  }

  public void requestHigh() {
    desiredRPM = 6000;
    requestHigh = true;
  }

  private void unsetAllRequests() {
    requestLow = false;
    requestMedium = false;
    requestHigh = false;
  }
}
