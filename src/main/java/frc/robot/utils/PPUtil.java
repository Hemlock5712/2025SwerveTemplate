package frc.robot.utils;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Utility class for comparing PathPlanner robot configurations and reporting discrepancies. This
 * class helps identify mismatches between two robot configurations by comparing various physical
 * and mechanical properties, raising alerts when differences are found.
 */
public final class PPUtil {
  /** Alert triggered when PathPlanner configuration fails to load */
  private static final Alert BAD_CONFIG = new Alert("Config Couldn't be loaded", AlertType.kError);
  /** Alert triggered when PathPlanner GUI fails to load */
  private static final Alert BAD_GUI = new Alert("GUI Couldn't be loaded", AlertType.kError);
  // Alert instances for different configuration mismatches
  /** Alert for moment of inertia configuration mismatches */
  private static final Alert MOI_ALERT = new Alert("MOI Config Mismatch", AlertType.kError);
  /** Alert for robot mass configuration mismatches */
  private static final Alert MASS_ALERT = new Alert("Mass Config Mismatch", AlertType.kError);
  /** Alert for torque friction configuration mismatches */
  private static final Alert TORQUE_ALERT = new Alert("Torque Friction Mismatch", AlertType.kError);
  /** Alert for drive current limit configuration mismatches */
  private static final Alert CURRENT_ALERT =
      new Alert("Drive Current Limit Mismatch", AlertType.kError);
  /** Alert for drive motor configuration mismatches */
  private static final Alert MOTOR_ALERT =
      new Alert("Drive Motor Config Mismatch", AlertType.kError);
  /** Alert for maximum drive velocity configuration mismatches */
  private static final Alert VELOCITY_ALERT =
      new Alert("Max Drive Velocity Mismatch", AlertType.kError);
  /** Alert for wheel coefficient of friction mismatches */
  private static final Alert COF_ALERT = new Alert("Wheel COF Mismatch", AlertType.kError);
  /** Alert for wheel radius configuration mismatches */
  private static final Alert RADIUS_ALERT = new Alert("Wheel Radius Mismatch", AlertType.kError);
  /** Alert for swerve module location mismatches */
  private static final Alert LOCATION_ALERT =
      new Alert("Module Location Mismatch", AlertType.kError);

  // Private constructor to prevent instantiation
  private PPUtil() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /**
   * Compares two PathPlanner robot configurations and raises alerts for any mismatches. This method
   * checks various robot parameters including physical properties (mass, MOI), drive system
   * configurations (current limits, motor settings), and module properties (wheel characteristics,
   * locations).
   *
   * @param config1 The first robot configuration to compare
   * @param config2 The second robot configuration to compare
   * @throws NullPointerException if either config is null
   */
  public static void compareConfigs(RobotConfig config1, RobotConfig config2) {
    validateConfigs(config1, config2);

    comparePhysicalProperties(config1, config2);
    compareDriveSystemConfig(config1, config2);
    compareWheelProperties(config1, config2);
    compareModuleLocations(config1, config2);
  }

  /**
   * Validates that both configurations are non-null and have matching module counts. Sets
   * BAD_CONFIG alert if validation fails.
   *
   * @param config1 First configuration to validate
   * @param config2 Second configuration to validate
   */
  private static void validateConfigs(RobotConfig config1, RobotConfig config2) {
    if (config1 == null
        || config2 == null
        || config1.moduleLocations.length != config2.moduleLocations.length) {
      BAD_CONFIG.set(true);
    }
  }

  /** Sets the BAD_GUI alert to indicate that the PathPlanner GUI failed to load. */
  public static void badGUI() {
    BAD_GUI.set(true);
  }

  /**
   * Compares physical properties between two robot configurations. Checks moment of inertia, mass,
   * and torque friction, raising alerts for any mismatches.
   *
   * @param config1 First configuration to compare
   * @param config2 Second configuration to compare
   */
  private static void comparePhysicalProperties(RobotConfig config1, RobotConfig config2) {
    if (config1.MOI != config2.MOI) {
      MOI_ALERT.setText(String.format("MOI: %.2f vs %.2f", config1.MOI, config2.MOI));
      MOI_ALERT.set(true);
    }

    if (config1.massKG != config2.massKG) {
      MASS_ALERT.setText(String.format("Mass: %.2f vs %.2f kg", config1.massKG, config2.massKG));
      MASS_ALERT.set(true);
    }

    if (config1.maxTorqueFriction != config2.maxTorqueFriction) {
      TORQUE_ALERT.setText(
          String.format(
              "Torque Friction: %.2f vs %.2f",
              config1.maxTorqueFriction, config2.maxTorqueFriction));
      TORQUE_ALERT.set(true);
    }
  }

  /**
   * Compares drive system configurations between two robot configurations. Checks current limits,
   * motor configurations, and maximum velocity, raising alerts for mismatches.
   *
   * @param config1 First configuration to compare
   * @param config2 Second configuration to compare
   */
  private static void compareDriveSystemConfig(RobotConfig config1, RobotConfig config2) {
    if (config1.moduleConfig.driveCurrentLimit != config2.moduleConfig.driveCurrentLimit) {
      CURRENT_ALERT.setText(
          String.format(
              "Drive Current Limit: %.2f vs %.2f",
              config1.moduleConfig.driveCurrentLimit, config2.moduleConfig.driveCurrentLimit));
      CURRENT_ALERT.set(true);
    }

    if (!config1.moduleConfig.driveMotor.equals(config2.moduleConfig.driveMotor)) {
      MOTOR_ALERT.setText("Drive Motor configurations differ");
      MOTOR_ALERT.set(true);
    }

    if (config1.moduleConfig.maxDriveVelocityMPS != config2.moduleConfig.maxDriveVelocityMPS) {
      VELOCITY_ALERT.setText(
          String.format(
              "Max Drive Velocity: %.2f vs %.2f m/s",
              config1.moduleConfig.maxDriveVelocityMPS, config2.moduleConfig.maxDriveVelocityMPS));
      VELOCITY_ALERT.set(true);
    }
  }

  /**
   * Compares wheel properties between two robot configurations. Checks coefficient of friction and
   * wheel radius, raising alerts for any mismatches.
   *
   * @param config1 First configuration to compare
   * @param config2 Second configuration to compare
   */
  private static void compareWheelProperties(RobotConfig config1, RobotConfig config2) {
    if (config1.moduleConfig.wheelCOF != config2.moduleConfig.wheelCOF) {
      COF_ALERT.setText(
          String.format(
              "Wheel COF: %.2f vs %.2f",
              config1.moduleConfig.wheelCOF, config2.moduleConfig.wheelCOF));
      COF_ALERT.set(true);
    }

    if (config1.moduleConfig.wheelRadiusMeters != config2.moduleConfig.wheelRadiusMeters) {
      RADIUS_ALERT.setText(
          String.format(
              "Wheel Radius: %.3f vs %.3f m",
              config1.moduleConfig.wheelRadiusMeters, config2.moduleConfig.wheelRadiusMeters));
      RADIUS_ALERT.set(true);
    }
  }

  /**
   * Compares swerve module locations between two robot configurations. Checks if module positions
   * match, raising an alert with detailed differences if they don't.
   *
   * @param config1 First configuration to compare
   * @param config2 Second configuration to compare
   */
  private static void compareModuleLocations(RobotConfig config1, RobotConfig config2) {
    StringBuilder locationDifferences = new StringBuilder();
    boolean hasLocationDifferences = false;

    // Compare module locations
    for (int i = 0; i < config1.moduleLocations.length; i++) {
      if (!config1.moduleLocations[i].equals(config2.moduleLocations[i])) {
        // Append the difference to the alert message
        locationDifferences.append(
            String.format(
                "Module %d: %s vs %s | ",
                i, config1.moduleLocations[i].toString(), config2.moduleLocations[i].toString()));
        // Set the alert flag
        hasLocationDifferences = true;
      }
    }
    // If there are differences, set the alert
    if (hasLocationDifferences) {
      LOCATION_ALERT.setText(locationDifferences.toString());
      LOCATION_ALERT.set(true);
    }
  }
}
