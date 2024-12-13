// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  private final SwerveRequest.FieldCentric drive;
  //   private final SwerveSetpointGen drive;
  // private final ProfiledFieldCentricFacingAngle drive;

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

        //        new Vision(
        //            drivetrain::addVisionData,
        //            new VisionIOLimelight("limelight-fl", drivetrain::getVisionParameters),
        //            new VisionIOLimelight("limelight-fr", drivetrain::getVisionParameters),
        //            new VisionIOLimelight("limelight-bl", drivetrain::getVisionParameters),
        //            new VisionIOLimelight("limelight-br", drivetrain::getVisionParameters));
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
        break;
    }

    // Default CTRE Swerve Drive Code
    drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(0.1))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Custom Swerve Request that use PathPlanner Setpoint Generator. You will need to tune
    // PP_CONFIG for this
    // drive =
    //     new SwerveSetpointGen(drivetrain.getChassisSpeeds(), drivetrain.getModuleStates())
    //         .withDeadband(MaxSpeed.times(0.1))
    //         .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // If you use this you will need to disable the right joystick
    //   drive =
    //       new ProfiledFieldCentricFacingAngle(
    //               new TrapezoidProfile.Constraints(
    //                   Constants.MaxAngularRate.baseUnitMagnitude(),
    //                   Constants.MaxAngularRate.div(0.25).baseUnitMagnitude()))
    //           .withDeadband(MaxSpeed.times(0.1))
    //           .withTargetDirection(Rotation2d.fromDegrees(45))
    //           .withRotationalDeadband(
    //               Constants.MaxAngularRate.times(0.1).baseUnitMagnitude()) // Add a 10% deadband
    //           .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    configureBindings();
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
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
                            joystick.getRightY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(joystick.getRightX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .getLeftX())))); // Drive counterclockwise with negative X (left)
    // .withRotation(drivetrain.getRotation())));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
