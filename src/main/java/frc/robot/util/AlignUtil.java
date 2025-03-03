package frc.robot.util;

// Java
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.geometry.Pose2d;

public class AlignUtil {
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotationController;

    public AlignUtil() {
        this.xController = new ProfiledPIDController(
            AutoConstants.kPXYController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                AutoConstants.kPathConstraints.maxVelocityMPS(),
                AutoConstants.kPathConstraints.maxAccelerationMPSSq()
            )
        );

        this.yController = new ProfiledPIDController(
            AutoConstants.kPXYController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                AutoConstants.kPathConstraints.maxVelocityMPS(),
                AutoConstants.kPathConstraints.maxAccelerationMPSSq()
            )
        );

        this.rotationController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(180),
                Units.degreesToRadians(180)
            )
        );

        rotationController.setIZone(Units.degreesToRadians(20));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ProfiledPIDController getXController() {
        return xController;
    }

    public ProfiledPIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getRotationController() {
        return rotationController;
    }

    public void resetControllers(Pose2d currentPose) {
        xController.reset(currentPose.getX(), 0);
        yController.reset(currentPose.getY(), 0);
        rotationController.reset(currentPose.getRotation().getRadians(), 0);
    }

    /**
     * A simple container for controller outputs.
     */
    public static class ControllerOutput {
        public final double xVel;
        public final double yVel;
        public final double rotVel;

        public ControllerOutput(double xVel, double yVel, double rotVel) {
            this.xVel = xVel;
            this.yVel = yVel;
            this.rotVel = rotVel;
        }
    }

    /**
     * Calculates the drive outputs based on the current and goal poses.
     *
     * @param currentPose The current robot pose.
     * @param goalPose    The target robot pose.
     * @return ControllerOutput containing velocities for X, Y, and rotation.
     */
    public ControllerOutput calculate(Pose2d currentPose, Pose2d goalPose) {
        double xFeedback = xController.calculate(currentPose.getX(), goalPose.getX());
        double yFeedback = yController.calculate(currentPose.getY(), goalPose.getY());
        double rotFeedback = rotationController.calculate(
            currentPose.getRotation().getRadians(),
            goalPose.getRotation().getRadians());

        double xFF = xController.getSetpoint().velocity;
        double yFF = yController.getSetpoint().velocity;
        double rotFF = rotationController.getSetpoint().velocity;

        double xVel = xFF + xFeedback;
        double yVel = yFF + yFeedback;
        double rotVel = rotFF + rotFeedback;

        if (Math.abs(currentPose.getX() - goalPose.getX()) < 0.025) {
            xVel = 0;
        }
        if (Math.abs(currentPose.getY() - goalPose.getY()) < 0.025) {
            yVel = 0;
        }
        if (Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees()) < 5) {
            rotVel = 0;
        }

        return new ControllerOutput(xVel, yVel, rotVel);
    }
}