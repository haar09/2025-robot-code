// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  
  public static final class AutoConstants {
    public static final double kRobotWeight = 0;
    public static final double kMOI = 0;

    public static final double kMaxSpeedCentimetersPerSecond = 4.5;
    public static final double kPXYController = 10;
    public static final double kPThetaController = 7;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedCentimetersPerSecond, 3, Math.toRadians(540), Math.toRadians(720));
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCam1 = //OV9281 001
                new Transform3d(new Translation3d(0.035, 0.221, 0.260435),
                                new Rotation3d(0, Math.toRadians(9.9), Math.toRadians(90))
                                );
    public static final Transform3d kRobotToCam2 = //OV9281 002
                new Transform3d(new Translation3d(0.035, -0.221, 0),
                                new Rotation3d(0, Math.toRadians(9.9), Math.toRadians(-90))
                                );

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double linearStdDevBaseline = 0.02; // Centimeters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZError = 0.75;

    public static final Translation2d kReefCenter =
        new Translation2d(Units.inchesToMeters(176.746), kTagLayout.getFieldWidth() / 2.0);
  }

  public static class IntakeConstants {
    public static final double kIntakeDeadband = 0.2;

    /* PIVOT */

    public static final double kMotorToEncoder = 22;
    public static final double kEncoderToPivot = 3;
    public static final double kTotalRatio = kMotorToEncoder * kEncoderToPivot;

    public static final int kPivotMotorId = 13;
    public static final int kAbsoluteEncoderId = 3;

    public static final double kAbsoluteEncoderOffset = 10;
    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    static  {
      pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
      pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      pivotMotorConfig.Feedback.SensorToMechanismRatio = 66;
    }

    // set slot 0 gains
    private static final Slot0Configs pivotMotor_slot0Configs;
    private static final Slot1Configs pivotMotor_slot1Configs;

    static {
      pivotMotor_slot0Configs = pivotMotorConfig.Slot0;
      pivotMotor_slot0Configs.kS = 0.099758; // Add 0.25 V output to overcome static friction
      pivotMotor_slot0Configs.kV = 1; // A velocity target of 1 rps results in 0.12 V output
      pivotMotor_slot0Configs.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
      pivotMotor_slot0Configs.kG = 0.30035;
      pivotMotor_slot0Configs.kP = 25; // An error of 1 rps results in 0.11 V output
      pivotMotor_slot0Configs.kI = 0; // no output for integrated error
      pivotMotor_slot0Configs.kD = 2; // no output for error derivative 
      pivotMotor_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

      pivotMotor_slot1Configs = pivotMotorConfig.Slot1;
      pivotMotor_slot1Configs.kS = 0.099758; // Add 0.25 V output to overcome static friction
      pivotMotor_slot1Configs.kV = 8; // A velocity target of 1 rps results in 0.12 V output
      pivotMotor_slot1Configs.kA = 0.6769; // An acceleration of 1 rps/s requires 0.01 V output
      pivotMotor_slot1Configs.kG = 0.30035;
      pivotMotor_slot1Configs.kP = 41; // An error of 1 rps results in 0.11 V output
      pivotMotor_slot1Configs.kI = 0; // no output for integrated error
      pivotMotor_slot1Configs.kD = 8.1903; // no output for error derivative 
      pivotMotor_slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;

      pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 3;
      pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 5;
    }

    public static final Angle kAngleTolerance = Degrees.of(4);

    public static final Angle idleAngle = Degrees.of(113);
    public static final Angle initialAngle = Degrees.of(16.2);
    public static final Angle intakeAngle = Degrees.of(-0.45);
    public static final Angle feedAngle = Degrees.of(113);
    public static final Angle shootAngle = Degrees.of(100);
    public static final Angle algaeAngle = Degrees.of(79);
    public static final Angle elevatorAngle = Degrees.of(86);

    /* PIVOT END ROLLER START */

    public static final int kRollerLeftMotorId = 11;
    public static final int kRollerRightMotorId = 12; 
    
    public static final boolean kRollerLeftMotorReversed = false;
    public static final boolean kRollerRightMotorReversed = false;
  }

  public static class DeployerConstants{
    public static final int kRollersMotorId = 22;
    public static final int kOmnisMotorId = 23;

    public static TalonFXConfiguration deployerMotorConfig = new TalonFXConfiguration();

    static {
      deployerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
      elevatorMotorConfig.Slot0.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
      elevatorMotorConfig.Slot0.kA = 0.009; // An acceleration of 1 rps/s requires 0.01 V output
      elevatorMotorConfig.Slot0.kP = 0.2; // An error of 1 rps results in 0.11 V output
      elevatorMotorConfig.Slot0.kI = 0; // no output for integrated error
      elevatorMotorConfig.Slot0.kD = 0; // no output for error derivative 
      elevatorMotorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      
      elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
      elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 275;
    }

    //public static final Distance IDLE = Centimeters.of(1);
    public static final Distance IDLE = Centimeters.of(0);
    public static final Distance CORAL_L2_HEIGHT = Centimeters.of(0);
    public static final Distance CORAL_L3_HEIGHT = Centimeters.of(0);
    public static final Distance INTAKE_HEIGHT = Centimeters.of(3);
    
    public static final Distance kDistanceTolerance = Centimeters.of(1.5);
  
    public static final Distance MAX_HEIGHT = Centimeters.of(53);
  }
} 
