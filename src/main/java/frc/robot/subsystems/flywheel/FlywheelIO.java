package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelMotorConnected = false;

    public Angle flywheelPositionRads = Radians.of(0.0);
    public AngularVelocity flywheelVelocityRpm = RotationsPerSecond.of(60.0);
    public Voltage flywheelAppliedVolts = Volt.of(0.0);
    public Current flywheelSupplyCurrentAmps = Amps.of(0.0);
    public Current flywheelTorqueCurrentAmps = Amps.of(0.0);
    public Temperature flywheelTempCelsius = Celsius.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run flywheel at voltage. */
  public default void runVolts(double volts) {}

  /** Stop flywheel. */
  public default void stop() {}

  /** Run flywheel at velocity in rpm */
  default void runVelocity(double flywheelRpm, double flywheelFeedforward) {}

  /** Config PID values for motor */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for motor */
  default void setFF(double kS, double kV, double kA) {}

  /** Run flywheel at voltage */
  default void runCharacterizationFlywheel(double input) {}
}
