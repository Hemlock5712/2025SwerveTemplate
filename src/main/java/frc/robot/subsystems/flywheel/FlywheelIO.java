package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public Angle flywheelPosition = Radians.of(0.0);
    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);
    public Voltage flywheelAppliedVolts = Volt.of(0.0);
    public Current flywheelCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the launcher wheel at the specified voltage. */
  public default void setFlywheelVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
