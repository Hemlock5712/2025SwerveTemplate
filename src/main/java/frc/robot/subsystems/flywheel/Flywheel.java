package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  /* Setpoints */
  private double desiredRPM = 0.0;

  /* System states */
  public enum FlywheelState {
    IDLE,
    LOW,
    MEDIUM,
    HIGH
  }

  /* SysId routine for characterizing rotation. This is used to find PID gains for the flywheel motors. */
  private final SysIdRoutine m_sysIDRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(1), // Reduce dynamic step voltage to 1 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> runCharacterization(output), null, this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIDRoutineRotation;

  public Flywheel(FlywheelIO io) {
    System.out.println("[Init] Creating Flywheel");
    this.io = io;
    flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor", AlertType.kError);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
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
      systemState = nextSystemState;
    }
  }
  // Returns whether the shooter is at setpoint for the superstructure
  public boolean atSetpoint() {
    Logger.recordOutput("Flywheel/RPM error", desiredRPM - inputs.flywheelVelocityRpm.abs(RPM));
    return Math.abs(desiredRPM - inputs.flywheelVelocityRpm.abs(RPM))
        < FLYWHEEL_SETPOINT_TOLERANCE_RPM;
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void runCharacterization(Voltage input) {
    io.runCharacterizationFlywheel(input);
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
