// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveWheelCalibration;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.RealPhotonVision;
import frc.robot.subsystems.apriltagvision.SimPhotonVision;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double SlowSpeed = 0.1;
  private double SlowAngularRate = 0.25 * Math.PI;
  private double AngularRate = MaxAngularRate;
  private int controlMode = 0;
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want r-centric driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(drivetrain.getState());
  private Supplier<SwerveRequest> controlStyle;

  private void configureBindings() {
    updateControlStyle();

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake).withName("Swerve Brake"));

    joystick.leftBumper().onTrue(runOnce(() -> controlMode = 1).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));
    joystick.leftBumper().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    joystick.rightBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * SlowSpeed).andThen(() -> AngularRate = SlowAngularRate)
    .andThen(() -> drive.withDeadband(0.0).withRotationalDeadband(0.0)).andThen(
      () -> robotOriented.withDeadband(0.0).withRotationalDeadband(0.0)));
    joystick.rightBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)).andThen(() -> AngularRate = MaxAngularRate)
    .andThen(() -> drive.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1))
    .andThen(() -> robotOriented.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)));

    joystick.rightTrigger(0.3).onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    // HALILI TESTLER
    
    joystick.pov(0).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.pov(90).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    joystick.pov(180).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.pov(270).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.back().whileTrue(new SwerveWheelCalibration(drivetrain));
    // HALILI TESTLER BİTİŞ


    // reset the field-centric heading on menu button
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(state -> logger.telemeterize(state));

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle",
            () -> drivetrain.getModule(0).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity",
            () -> drivetrain.getModule(0).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle",
            () -> drivetrain.getModule(1).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity",
            () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getCurrentState().angle.getRadians(),
            null);
        builder.addDoubleProperty("Back Left Velocity",
            () -> drivetrain.getModule(2).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle",
            () -> drivetrain.getModule(3).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity",
            () -> drivetrain.getModule(3).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().Pose.getRotation().getRadians(), null);
      }
    });
  
    }

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    new LEDSubsystem();
    new ObjectDetection();

    if (Robot.isReal()) {
      new AprilTagVision(drivetrain::addVisionMeasurement,
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_001", VisionConstants.kRobotToCam1),
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_002", VisionConstants.kRobotToCam2));
    } else {
      new AprilTagVision(drivetrain::addVisionMeasurement,
                       new SimPhotonVision("Arducam_OV9281_USB_Camera_001", VisionConstants.kRobotToCam1, () -> drivetrain.getState().Pose),
                       new SimPhotonVision("Arducam_OV9281_USB_Camera_002", VisionConstants.kRobotToCam2, () -> drivetrain.getState().Pose));
    }

    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void updateControlStyle() {
    SmartDashboard.putNumber("Control Mode", controlMode);
    Logger.recordOutput("Drive/Control Mode", controlMode);
    switch (controlMode) {
      case 0:
        controlStyle = () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * AngularRate);
        break;
      case 1:
        controlStyle = () -> robotOriented.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * AngularRate);
        break;
    }
    try {
        drivetrain.getDefaultCommand().cancel();
    } catch(Exception e) {}
    drivetrain.setDefaultCommand(drivetrain.applyRequest(controlStyle).ignoringDisable(true).withName("Swerve Command "+controlMode));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
