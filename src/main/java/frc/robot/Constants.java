// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
} 
