// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
  public static final class Robotconstants {
    public static final boolean kTuningMode = true;
  }

  public static final class AutoConstants {
    public static final double kRobotWeight = 65;
    public static final double kMOI = 4.8;

    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final double kPXYController = 5;
    public static final double kPThetaController = 6;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 4, Math.toRadians(540), Math.toRadians(720));
      
    public static final PPHolonomicDriveController kAutoController = new PPHolonomicDriveController(
        // PID constants for translation
        new PIDConstants(4, 0, 0),
        // PID constants for rotation
        new PIDConstants(6, 0, 0)
    );    
    
    public static final PPHolonomicDriveController kAlignController = new PPHolonomicDriveController(
      // PID constants for translation
      new PIDConstants(kPXYController, 0, 0.3),
      // PID constants for rotation
      new PIDConstants(kPThetaController, 0, 0)
    );

    public static final double maxDistanceReefLineup = 1.5;
    
  }

  public static class VisionConstants {
    // fl 2 fr 3 bl 4 br 1
    public static final Transform3d kRobotToCam1 = //OV9281 001
                new Transform3d(new Translation3d(-0.23323, -0.275528, 0.204508),
                                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(-60))
                                );
    public static final Transform3d kRobotToCam2 = //OV9281 002
                new Transform3d(new Translation3d(0.23323, 0.275528, 0.204508),
                                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(120))
                                );
    public static final Transform3d kRobotToCam3 = //OV9281 003
                new Transform3d(new Translation3d(0.23323, -0.275528, 0.204508),
                                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(-116.8))
                                );
    public static final Transform3d kRobotToCam4 = //OV9281 004
                new Transform3d(new Translation3d(-0.23323, 0.275528, 0.204508),
                                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(63.2))
                                );

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static double linearStdDevBaseline = 0.08; // Centimeters
    public static double angularStdDevBaseline = Math.toRadians(4.5); // Radians

    public static double[] cameraStdDevFactors =
    new double[] {
      1.0,
      1.0,
      1.0,
      1.0
    };

    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZError = 0.12;

    public static final LoggedTunableNumber kTXController_P = new LoggedTunableNumber("ObjectDetection/tx_P", 0.04);
    public static final LoggedTunableNumber kTXController_D = new LoggedTunableNumber("ObjectDetection/tx_D", 0.008);
  }

  public static class IntakeConstants {
    public static final double kIntakeDeadband = 0.2;

    /* PIVOT */
    public static final int kPivotMotorId = 13;
    public static final int kAbsoluteEncoderId = 9;

    public static final double kAbsoluteEncoderOffset = -0.339;
    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    static  {
      pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
      pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      pivotMotorConfig.Feedback.SensorToMechanismRatio = 48;
    }

    public static class MotionProfileConstants {
      /*pivotMotor_slot0Configs = pivotMotorConfig.Slot0;
      pivotMotor_slot0Configs.kS = 0.099758; // Add 0.25 V output to overcome 3atic friction
      pivotMotor_slot0Configs.kV = 1; // A velocity target of 1 rps results in 0.12 V output
      pivotMotor_slot0Configs.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
      pivotMotor_slot0Configs.kG = 0.30035;
      pivotMotor_slot0Configs.kP = 25; // An error of 1 rps results in 0.11 V output
      pivotMotor_slot0Configs.kI = 0; // no output for integrated error
      pivotMotor_slot0Configs.kD = 2; // no output for error derivative 
      pivotMotor_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;*/

      public static final double kP = 0.13; // An error of 1 rps results in 0.11 V output
      public static final double kI = 0; // no output for integrated error
      public static final double kD = 0; // no output for error derivative 
    }

    public static final Angle kAngleTolerance = Degrees.of(3);

    public static final LoggedTunableNumber idleAngle = new LoggedTunableNumber("Intake/Idle", 145);
    public static final LoggedTunableNumber intakeAngle = new LoggedTunableNumber("Intake/Intake", -5);
    public static final LoggedTunableNumber feedAngle = new LoggedTunableNumber("Intake/Feed", 165);
    public static final LoggedTunableNumber shootAngle = new LoggedTunableNumber("Intake/Shoot", 95);
    public static final LoggedTunableNumber algaeAngle = new LoggedTunableNumber("Intake/Algae", 73);
    public static final LoggedTunableNumber elevatorAngle = new LoggedTunableNumber("Intake/Elevator", 102);    

    /* PIVOT END ROLLER START */

    public static final int kRollerMotorId = 12;
    
    public static final boolean kRollerMotorReversed = false;
  }

  public static class DeployerConstants{
    public static final int kRollersMotorId = 22;
    public static final int kOmnisMotorId = 23;

    public static TalonFXConfiguration deployerMotorConfig = new TalonFXConfiguration();

    static {
      deployerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      deployerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }
  }

  public static class ElevatorConstants{
    public static final double kElevatorRotToCm = 1.693;

    public static final int kLeftMotorId = 20;
    public static final int kRightMotorId = 21;

    public static TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();

    static {
      elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
      elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      elevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      //0.038 öbürü 0.019 
      //x=0.0285 y=0.0095
      elevatorMotorConfig.Slot0.kG = 0.285;
      elevatorMotorConfig.Slot0.kS = 0.095; // Add 0.25 V output to overcome static friction
      elevatorMotorConfig.Slot0.kV = 0.155; // A velocity target of 1 rps results in 0.12 V output
      elevatorMotorConfig.Slot0.kA = 0.002; // An acceleration of 1 rps/s requires 0.01 V output
      elevatorMotorConfig.Slot0.kP = 0.63; // An error of 1 rps results in 0.11 V output
      elevatorMotorConfig.Slot0.kI = 0; // no output for integrated error
      elevatorMotorConfig.Slot0.kD = 0; // no output for error derivative 
      elevatorMotorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      
      elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 350;
      elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 300;
    }

    public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Elevator/Idle", 0);
    public static final LoggedTunableNumber CORAL_L2_HEIGHT = new LoggedTunableNumber("Elevator/CORAL_L2_HEIGHT", 28.5);
    public static final LoggedTunableNumber CORAL_L3_HEIGHT = new LoggedTunableNumber("Elevator/CORAL_L3_HEIGHT", 48.5);
    public static final LoggedTunableNumber INTAKE_HEIGHT = new LoggedTunableNumber("Elevator/INTAKE_HEIGHT", 3);
    public static final LoggedTunableNumber SOURCE_HEIGHT = new LoggedTunableNumber("Elevator/SOURCE_HEIGHT", 14);
    
    public static final Distance kDistanceTolerance = Centimeters.of(0.5);
    public static final Distance MAX_HEIGHT = Centimeters.of(53);

    public static final double kL3Offset = 0.06;
  }


  public static class ClimbConstants{
    public static final int kClimbMotorId = 40;
    public static final TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
    static {
      climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      climbMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public static class AlgMechanismConstants{
    public static final int kRackMotorId = 31;
    //public static final int kRollerMotorId = 32;
  } 
}