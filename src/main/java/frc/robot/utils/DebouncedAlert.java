package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DebouncedAlert {
  private final Alert warningAlert;
  private final Alert errorAlert;
  private final Debouncer debounce;

  /**
   * Creates a new DebouncedAlert with separate warning and error messages.
   *
   * @param warningText Text to display for the warning condition
   * @param errorText Text to display for the error condition
   * @param debounceTime Time in seconds that the condition must be true before triggering error
   */
  public DebouncedAlert(String warningText, String errorText, double debounceTime) {
    this.warningAlert = new Alert("Alerts", warningText, AlertType.kWarning);
    this.errorAlert = new Alert("Alerts", errorText, AlertType.kError);
    this.debounce = new Debouncer(debounceTime);
  }

  /**
   * Sets warning and error states based on the current condition. Warning is set when condition is
   * true but hasn't exceeded debounce time. Error is set only after the debounce period if
   * condition remains true. Only one alert will be active at a time, with error taking precedence.
   *
   * @param condition The current state to evaluate
   */
  public void set(boolean condition) {
    boolean debouncedCondition = debounce.calculate(condition);

    if (debouncedCondition) {
      // If debounced condition is true, show error only
      errorAlert.set(true);
      warningAlert.set(false);
    } else if (condition) {
      // If immediate condition is true but not debounced, show warning only
      errorAlert.set(false);
      warningAlert.set(true);
    } else {
      // If condition is false, clear both
      clear();
    }
  }

  /** Clears both warning and error alerts. */
  public void clear() {
    warningAlert.set(false);
    errorAlert.set(false);
  }

  /**
   * @return The current warning state
   */
  public boolean getWarningState() {
    return warningAlert.get();
  }

  /**
   * @return The current error state
   */
  public boolean getErrorState() {
    return errorAlert.get();
  }
}
