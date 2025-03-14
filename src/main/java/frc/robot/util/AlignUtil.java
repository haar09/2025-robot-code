package frc.robot.util;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.geometry.Pose2d;
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
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d goalPose) {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        return mDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);
    }
}