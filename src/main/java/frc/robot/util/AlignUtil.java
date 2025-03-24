package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import frc.robot.Constants.AutoConstants;
import frc.robot.FieldConstants.Reef;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AlignUtil {
    private PPHolonomicDriveController mDriveController = AutoConstants.kAlignController;

    public AlignUtil() {
    }

    public void resetControllers(Pose2d currentPose, ChassisSpeeds curreentSpeeds) {
        mDriveController.reset(currentPose, curreentSpeeds);
    }

    /**
     * Calculates the drive outputs based on the current and goal poses.
     *
     * @param currentPose The current robot pose.
     * @param goalPose    The target robot pose.
     * @return ControllerOutput containing velocities for X, Y, and rotation.
     */
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d goalPose, boolean leftInstead) {
        // Final line up
        var offset = currentPose.relativeTo(goalPose);
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        Logger.recordOutput("AutoShoot/yDistance", offset.getY());
        Logger.recordOutput("AutoShoot/xDistance", offset.getX());
        double shiftIleriGeri =
            MathUtil.clamp(
                (xDistance / (Reef.faceLength * 2)) + ((yDistance - 0.3) / (Reef.faceLength * 4)),
                0.0,
                1.0);
        Logger.recordOutput("AutoShoot/shiftIleriGeri", shiftIleriGeri);
        goalPose = goalPose.transformBy(
            new Transform2d(
                0,
                (leftInstead ? -1 : 1) * shiftIleriGeri * AutoConstants.maxDistanceReefLineup,
                new Rotation2d()));
        Logger.recordOutput("AutoShoot/Goal Position", goalPose);

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        return mDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);
    }

    public ChassisSpeeds calculateVanilla(Pose2d currentPose, Pose2d goalPose) {
        Logger.recordOutput("AutoShoot/Goal Position", goalPose);

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        return mDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);
    }
}