package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Disconnected alerts
  private final Alert flywheelDisconnectedAlert;

  public Flywheel(FlywheelIO io) {
    System.out.println("[Init] Creating Flywheel");
    this.io = io;
    flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    // Update alerts
    flywheelDisconnectedAlert.set(!inputs.flywheelMotorConnected);
  }
}
