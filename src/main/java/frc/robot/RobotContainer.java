package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOCTRE;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  public final Flywheel flywheel;

  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain =
            new Drive(
                currentDriveTrain,
                new ModuleIOCTRE(currentDriveTrain.getModule(0)),
                new ModuleIOCTRE(currentDriveTrain.getModule(1)),
                new ModuleIOCTRE(currentDriveTrain.getModule(2)),
                new ModuleIOCTRE(currentDriveTrain.getModule(3)));

        new Vision(
            drivetrain::addVisionData,
            new VisionIOLimelight("limelight-fl", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-fr", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-bl", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-br", drivetrain::getVisionParameters));

        flywheel = new Flywheel(new FlywheelIOCTRE());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain =
            new Drive(
                currentDriveTrain,
                new ModuleIOCTRE(currentDriveTrain.getModule(0)),
                new ModuleIOCTRE(currentDriveTrain.getModule(1)),
                new ModuleIOCTRE(currentDriveTrain.getModule(2)),
                new ModuleIOCTRE(currentDriveTrain.getModule(3)));

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "Front Camera",
                new Transform3d(
                    new Translation3d(0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Back Camera",
                new Transform3d(
                    new Translation3d(-0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Left Camera",
                new Transform3d(
                    new Translation3d(0.0, 0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Right Camera",
                new Transform3d(
                    new Translation3d(0.0, -0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))),
                drivetrain::getVisionParameters));

        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain =
            new Drive(
                new DriveIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick.getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(-joystick.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .getRightX())))); // Drive counterclockwise with negative X (left)

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. You will need to tune
    // PP_CONFIG first
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.1))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    joystick
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -joystick.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
                        .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
                        .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Custom Swerve Request that use ProfiledFieldCentricFacingAngle. Allows you to face specific
    // direction while driving
    ProfiledFieldCentricFacingAngle driveFacingAngle =
        new ProfiledFieldCentricFacingAngle(
                new TrapezoidProfile.Constraints(
                    Constants.MaxAngularRate.baseUnitMagnitude(),
                    Constants.MaxAngularRate.div(0.25).baseUnitMagnitude()))
            .withDeadband(MaxSpeed.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Set PID for ProfiledFieldCentricFacingAngle
    driveFacingAngle.HeadingController.setPID(7, 0, 0);
    joystick
        .y()
        .whileTrue(
            drivetrain
                .runOnce(() -> driveFacingAngle.resetProfile(drivetrain.getRotation()))
                .andThen(
                    drivetrain.applyRequest(
                        () ->
                            driveFacingAngle
                                .withVelocityX(
                                    MaxSpeed.times(
                                        -joystick
                                            .getLeftY())) // Drive forward with negative Y (forward)
                                .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
                                .withTargetDirection(
                                    new Rotation2d(
                                        -joystick.getRightY(), -joystick.getRightX())))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    joystick
        .leftBumper()
        .whileTrue(
            Commands.startEnd(() -> flywheel.requestLow(), () -> flywheel.requestIdle(), flywheel));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
