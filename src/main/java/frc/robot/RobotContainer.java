// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlgMechanismCmd;
import frc.robot.commands.AutoBranchandShootL1;
import frc.robot.commands.AutoBranchandShootL23;
import frc.robot.commands.DpadBranchandShootL23;
import frc.robot.commands.ClimbCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.RealPhotonVision;
import frc.robot.subsystems.apriltagvision.SimPhotonVision;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.subsystems.superstructure.StateManager.State;
import frc.robot.subsystems.superstructure.algMechanism.AlgMechanism;
import frc.robot.subsystems.superstructure.deployer.Deployer;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NetworkTablesAgent;

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

  private final Telemetry logger = new Telemetry(drivetrain.getState());
  private Supplier<SwerveRequest> controlStyle;

  private void configureBindings() {
    updateControlStyle();
 
    joystick.y().onTrue(stateManager.setStateCommand(State.TEST));
    joystick.y().onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.a().onTrue(stateManager.setStateCommand(State.FEED));
    joystick.a().onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.b().onTrue(stateManager.setStateCommand(State.L1));
    joystick.b().onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.x().onTrue(stateManager.setStateCommand(State.SOURCE_INTAKE));
    joystick.x().onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.leftBumper().onTrue(runOnce(() -> controlMode = 1).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));
    joystick.leftBumper().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    joystick.rightBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * SlowSpeed).andThen(() -> AngularRate = SlowAngularRate)
    .andThen(() -> drive.withDeadband(0.0).withRotationalDeadband(0.0)).andThen(
      () -> robotOriented.withDeadband(0.0).withRotationalDeadband(0.0)));
    joystick.rightBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)).andThen(() -> AngularRate = MaxAngularRate)
    .andThen(() -> drive.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1))
    .andThen(() -> robotOriented.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)));

    joystick.leftTrigger(IntakeConstants.kIntakeDeadband).whileTrue(stateManager.setStateCommand(State.CORAL_INTAKE));
    joystick.leftTrigger(IntakeConstants.kIntakeDeadband).onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.rightTrigger(IntakeConstants.kIntakeDeadband).whileTrue(stateManager.setStateCommand(State.ALGAE_INTAKE));
    joystick.rightTrigger(IntakeConstants.kIntakeDeadband).onFalse(stateManager.setStateCommand(State.IDLE));

    joystick.povRight().whileTrue(new DpadBranchandShootL23(false, drivetrain, stateManager, operator));
    joystick.povLeft().whileTrue(new DpadBranchandShootL23(true, drivetrain, stateManager, operator));

    // HALILI TESTLER
    
    /*joystick.pov(0).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.pov(90).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    joystick.pov(180).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.pov(270).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));*/
    //joystick.back().whileTrue(new SwerveWheelCalibration(drivetrain));
    // HALILI TESTLER BİTİŞ


    // reset the field-centric heading on menu button
    joystick.start().onTrue(runOnce(() -> drivetrain.seedFieldCentric()));
    joystick.back().onTrue(runOnce(() -> intake.intakePivot.manualEncoderReset()));

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
  
   new Trigger(() -> !networkTablesAgent.buttonValue.get().contentEquals("N") && networkTablesAgent.triggerValue.get() == 1)
   .whileTrue(new AutoBranchandShootL1(networkTablesAgent, drivetrain, stateManager));

   new Trigger(() -> !networkTablesAgent.buttonValue.get().contentEquals("N") && networkTablesAgent.triggerValue.get() != 1)
   .whileTrue(new AutoBranchandShootL23(networkTablesAgent, drivetrain, stateManager));
   
 /*new Trigger(() -> !networkTablesAgent.upDownValue.get().contentEquals("N") && networkTablesAgent.elevatorClimbSwitchValue.get().contentEquals("E"))
  .whileTrue(new ElevatorCmd(elevator, () -> networkTablesAgent.upDownValue.get()));

  operator.povUp().whileTrue(new ElevatorCmd(elevator, () -> "U", intake).alongWith(intake.setStatecCommand(IntakeState.ELEVATOR)));
  operator.povDown().whileTrue(new ElevatorCmd(elevator, () -> "D", intake).alongWith(intake.setStatecCommand(IntakeState.ELEVATOR)));*/

  new NetworkButton("Arduino", "Override")
      .whileTrue(new InstantCommand(() -> {
          double trigger = networkTablesAgent.triggerValue.get();
          switch ((int) trigger) {
              case 1:
                  stateManager.setStateCommand(StateManager.State.L1).schedule();
                  break;
              case 2:
                  stateManager.setStateCommand(StateManager.State.L2_RIGHT).schedule();
                  break;
              case 3:
                  stateManager.setStateCommand(StateManager.State.L3_RIGHT).schedule();
                  break;
              default:
              break;
          }
      }));
      
      new NetworkButton("Arduino", "Override").onFalse(stateManager.setStateCommand(State.IDLE));
    
    }

  private LoggedDashboardChooser<Command> autoChooser;
  private Intake intake;
  private Elevator elevator;
  private Deployer deployer;
  private StateManager stateManager;
  private NetworkTablesAgent networkTablesAgent;
  private AlgMechanism algMechanism;
  private Climb climb;

  public RobotContainer() {
    new ObjectDetection();
    intake = new Intake();
    elevator = Elevator.create();
    deployer = new Deployer();
    stateManager = new StateManager(deployer, intake, elevator);
    networkTablesAgent = new NetworkTablesAgent();
    algMechanism = AlgMechanism.create();
    climb = Climb.create();

    if (Robot.isReal()) {
      new AprilTagVision(drivetrain,
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_001", VisionConstants.kRobotToCam1),
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_002", VisionConstants.kRobotToCam2),
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_03", VisionConstants.kRobotToCam3),
                       new RealPhotonVision("Arducam_OV9281_USB_Camera_04", VisionConstants.kRobotToCam4));
    } else {
      new AprilTagVision(drivetrain,
                       new SimPhotonVision("Arducam_OV9281_USB_Camera_001", VisionConstants.kRobotToCam1, () -> drivetrain.getState().Pose),
                       new SimPhotonVision("Arducam_OV9281_USB_Camera_002", VisionConstants.kRobotToCam2, () -> drivetrain.getState().Pose));
    }

    algMechanism.setDefaultCommand(
      new AlgMechanismCmd(algMechanism, () -> joystick.povUp().getAsBoolean(), () -> joystick.povDown().getAsBoolean()));
    climb.setDefaultCommand(
        new ClimbCmd(() -> operator.povRight().getAsBoolean(), () -> operator.povLeft().getAsBoolean(), climb));
    
    NamedCommands.registerCommand("L1", stateManager.setStateCommand(State.L1)); //YAZ L1
    NamedCommands.registerCommand("Source", stateManager.setStateCommand(State.SOURCE_INTAKE).until(()->SmartDashboard.getBoolean("Deployer Ready", false))
    .withTimeout(3)
    .andThen(stateManager.setStateCommand(State.IDLE)));

    configureBindings();

    Logger.recordOutput("ScoringPosition", AllianceFlipUtil.apply(FieldConstants.Reef.scoringPositions2d.get(1).get(FieldConstants.ReefLevel.L23)));

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
    if (autoChooser.getSendableChooser().getSelected() != "None") {
      if (SmartDashboard.getBoolean("Flip Horizontally", false)) {
        return new PathPlannerAuto(autoChooser.getSendableChooser().getSelected(), true);
      }
    }
    return autoChooser.get();
  }
}
