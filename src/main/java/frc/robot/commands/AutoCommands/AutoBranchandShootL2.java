package frc.robot.commands.AutoCommands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.GlobalVariables;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.util.AlignUtil;
import frc.robot.util.AllianceFlipUtil;

public class AutoBranchandShootL2 extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;
    private final AlignUtil alignUtil = new AlignUtil();

    private Pose2d goalPosition;
    private boolean leftInstead = false;
    private boolean leftBranch = false;

    public AutoBranchandShootL2(boolean leftBranch, CommandSwerveDrivetrain drivetrain, StateManager stateManager) {
        this.drivetrain = drivetrain;
        this.leftBranch = leftBranch;
        this.stateManager = stateManager;
    }

    @Override
    public void initialize() {
        state = State.TRAVELLING;
        Pose2d currentPose = drivetrain.getState().Pose;

        alignUtil.resetControllers(currentPose, drivetrain.getState().Speeds);

        if (!leftBranch) {
            goalPosition = Reef.scoringPositions2d.get(FieldConstants.findClosestReefside(currentPose)*2).get(ReefLevel.L23);
        } else {
            goalPosition = Reef.scoringPositions2d.get(FieldConstants.findClosestReefside(currentPose)*2+1).get(ReefLevel.L23);
        }

        double angleDifference = goalPosition.getRotation().plus(Rotation2d.kCCW_90deg).minus(currentPose.getRotation()).getDegrees();

            if ((GlobalVariables.getInstance().alliance == Alliance.Blue && angleDifference < 0) ||
            (GlobalVariables.getInstance().alliance != Alliance.Blue && angleDifference > 0)) {
                leftInstead = true;
                GlobalVariables.getInstance().alignStatus = 2;
                goalPosition = goalPosition.transformBy(new Transform2d(-0.36, 0, new Rotation2d(Units.degreesToRadians(180))));
            } else {
                leftInstead = false;
                GlobalVariables.getInstance().alignStatus = 1;
            }

            goalPosition = AllianceFlipUtil.apply(goalPosition);
        }

    private enum State{
        TRAVELLING,
        READY
    }

    private State state = State.TRAVELLING;

    @Override
    public void execute() {
        switch (state){
            case TRAVELLING:
        Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition, leftInstead);
    
            PPHolonomicDriveController.overrideXFeedback(() -> { return output.vxMetersPerSecond; });
            PPHolonomicDriveController.overrideYFeedback(() -> { return output.vyMetersPerSecond; });
            PPHolonomicDriveController.overrideRotationFeedback(() -> { return output.omegaRadiansPerSecond; });
    
        Logger.recordOutput("AutoShoot/Left Distance", currentPose.minus(goalPosition).getTranslation().getNorm());
        if (currentPose.minus(goalPosition).getTranslation().getNorm() < 0.02
        && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 2){
            setState(State.READY);
        }
        break;
        case READY:
                if (leftInstead) {
                    stateManager.state = StateManager.State.L2_LEFT;
                } else {
                    stateManager.state = StateManager.State.L2_RIGHT;
                }
            break;
    }
    }

    @Override
    public void end(boolean interrupted) {
        stateManager.state = StateManager.State.IDLE;
        GlobalVariables.getInstance().alignStatus = 0;
        PPHolonomicDriveController.clearFeedbackOverrides();;
    }

    @Override
    public boolean isFinished() {
        //return !stateManager.deployer.deployerBeamBreak.left_value && !stateManager.deployer.deployerBeamBreak.right_value;
        return false;
    }

    private void setState(State state){
        this.state = state;
    }
}