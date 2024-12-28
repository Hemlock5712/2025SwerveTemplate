package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A customizable Xbox controller that provides various input scaling options. This class extends
 * CommandXboxController to provide additional functionality for scaling joystick inputs using
 * different response curves.
 *
 * <p>The controller supports different input response types through the {@link
 * TunableControllerType} enum, allowing for LINEAR, QUADRATIC, and CUBIC response curves. This can
 * be useful for:
 *
 * <ul>
 *   <li>Fine control at low speeds (higher order curves)
 *   <li>Linear response for predictable input (linear curve)
 *   <li>Custom response curves for specific driving scenarios
 * </ul>
 *
 * <p>Usage example:
 *
 * <pre>
 * TunableController controller = new TunableController(0)
 *     .withControllerType(TunableControllerType.CUBIC);
 * double customeLeftX = controller.customLeft().getX();
 * </pre>
 */
public class TunableController extends CommandXboxController {
  private TunableControllerType type = TunableControllerType.LINEAR;

  /** Minimum magnitude threshold for considering stick input non-zero */
  private static final double DEADBAND_THRESHOLD = 1e-6;

  /**
   * Constructs a new TunableController.
   *
   * @param port The port index on the Driver Station (0-5) that the controller is plugged into.
   */
  public TunableController(int port) {
    super(port);
  }

  /**
   * Gets the scaled input values from the left stick.
   *
   * @return A Translation2d representing the scaled x and y values from the left stick. The
   *     magnitude is guaranteed to be between 0 and 1.
   */
  public Translation2d customLeft() {
    return getCustom(getLeftX(), getLeftY());
  }

  /**
   * Gets the scaled input values from the right stick.
   *
   * @return A Translation2d representing the scaled x and y values from the right stick. The
   *     magnitude is guaranteed to be between 0 and 1.
   */
  public Translation2d customRight() {
    return getCustom(getRightX(), getRightY());
  }

  /**
   * Processes raw stick inputs and applies the current scaling curve.
   *
   * @param x The raw X axis input, expected to be between -1.0 and 1.0
   * @param y The raw Y axis input, expected to be between -1.0 and 1.0
   * @return A Translation2d representing the scaled input values
   */
  private Translation2d getCustom(double x, double y) {
    // Create Translation2d from raw inputs
    Translation2d input = new Translation2d(x, y);

    // Return zero Translation2d if input is below threshold
    if (input.getNorm() < DEADBAND_THRESHOLD) {
      return new Translation2d();
    }

    // Get the input angle and magnitude
    Rotation2d angle = input.getAngle();
    double magnitude = input.getNorm();

    // Scale the magnitude while preserving direction
    double scaledMagnitude = Math.pow(magnitude, type.exponent);
    scaledMagnitude = Math.min(scaledMagnitude, 1.0);

    // Convert back to x,y coordinates
    return new Translation2d(scaledMagnitude * angle.getCos(), scaledMagnitude * angle.getSin());
  }

  /**
   * Defines different types of input response curves. Each type provides a different relationship
   * between input and output magnitudes while preserving the input direction.
   */
  public enum TunableControllerType {
    /** Linear scaling (1:1 relationship) */
    LINEAR(1.0),

    /** Quadratic scaling (more precise at low inputs) */
    QUADRATIC(2.0),

    /** Cubic scaling (even more precise at low inputs) */
    CUBIC(3.0);

    private final double exponent;

    TunableControllerType(double exponent) {
      this.exponent = exponent;
    }
  }

  /**
   * Sets the controller's input response type.
   *
   * @param type The desired {@link TunableControllerType} for scaling inputs
   * @return This controller instance for method chaining
   */
  public TunableController withControllerType(TunableControllerType type) {
    this.type = type;
    return this;
  }
}
