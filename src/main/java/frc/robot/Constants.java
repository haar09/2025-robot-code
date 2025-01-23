// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public final class Constants {
  
  public static final class AutoConstants {
    public static final double kRobotWeight = 0;
    public static final double kMOI = 0;

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kPXYController = 10;
    public static final double kPThetaController = 7;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 3, Math.toRadians(540), Math.toRadians(720));
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCam1 = //OV9281 001
                new Transform3d(new Translation3d(0, 0, 0),
                                new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
                                );
    public static final Transform3d kRobotToCam2 = //OV9281 002
                new Transform3d(new Translation3d(0, 0, 0),
                                new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
                                );

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZError = 0.75;
  }

  public static class IntakeConstants {
    /* PIVOT */

    public static final double kMinIntakeAngleDegrees = 0;
    public static final double kMaxIntakeAngleDegrees = 0;

    public static final int kPivotMotorId = 0;
    public static final int kAbsoluteEncoderId = 0;

    public static final double kAbsoluteEncoderOffset = 0;
    public static final boolean kPivotMotorReversed = true;
    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    static  {
      pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
      pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    private static final FeedbackConfigs pivotMotor_feedbackConfigs;

    static {
      pivotMotor_feedbackConfigs = pivotMotorConfig.Feedback;
      pivotMotor_feedbackConfigs.RotorToSensorRatio = 0;
      pivotMotor_feedbackConfigs.SensorToMechanismRatio = 0;
      pivotMotor_feedbackConfigs.FeedbackRotorOffset = 0;
    }

    // set slot 0 gains
    private static final Slot0Configs pivotMotor_slot0Configs;

    static {
      pivotMotor_slot0Configs = pivotMotorConfig.Slot0;
      pivotMotor_slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
      pivotMotor_slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
      pivotMotor_slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
      pivotMotor_slot0Configs.kP = 0; // An error of 1 rps results in 0.11 V output
      pivotMotor_slot0Configs.kI = 0; // no output for integrated error
      pivotMotor_slot0Configs.kD = 0; // no output for error derivative 
      pivotMotor_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    }

    public static final Angle idleAngle = Degrees.of(0);
    public static final Angle initialAngle = Degrees.of(0);
    public static final Angle intakeAngle = Degrees.of(0);
    public static final Angle feedAngle = Degrees.of(0);
    public static final Angle sourceAngle = Degrees.of(0);
    public static final Angle shootAngle = Degrees.of(0);

    /* PIVOT END ROLLER START */

    public static final int kRollerLeftMotorId = 0;
    public static final int kRollerRightMotorId = 0; 
    
    public static final boolean kRollerLeftMotorReversed = false;
    public static final boolean kRollerRightMotorReversed = false;

    public static final double kRollerLeftMotorSpeed = 0;
    public static final double kRollerRightMotorSpeed = 0;
  }
} 
