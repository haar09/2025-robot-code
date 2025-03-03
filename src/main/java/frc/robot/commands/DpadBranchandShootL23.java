package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.GlobalVariables;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.util.AlignUtil;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AlignUtil.ControllerOutput;

public class DpadBranchandShootL23 extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;
    private final CommandXboxController operator;
    private final AlignUtil alignUtil = new AlignUtil();

    private Pose2d goalPosition;
    private boolean leftInstead = false;
    private boolean leftBranch = false;

    public DpadBranchandShootL23(boolean leftBranch, CommandSwerveDrivetrain drivetrain, StateManager stateManager, CommandXboxController operator) {
        this.drivetrain = drivetrain;
        this.leftBranch = leftBranch;
        this.stateManager = stateManager;
        this.operator = operator;
    }

    @Override
    public void initialize() {
        state = State.TRAVELLING;
        Pose2d currentPose = drivetrain.getState().Pose;

        alignUtil.resetControllers(currentPose);

        if (!leftBranch) {
            goalPosition = Reef.scoringPositions2d.get(FieldConstants.findClosestReefside(currentPose)*2).get(ReefLevel.L23);
        } else {
            goalPosition = Reef.scoringPositions2d.get(FieldConstants.findClosestReefside(currentPose)*2+1).get(ReefLevel.L23);
        }


        double angleDifference = goalPosition.getRotation().plus(Rotation2d.kCCW_90deg).minus(currentPose.getRotation()).getDegrees();

            if ((GlobalVariables.getInstance().alliance == Alliance.Blue && angleDifference < 0) ||
            (GlobalVariables.getInstance().alliance != Alliance.Blue && angleDifference > 0)) {
                leftInstead = true;
                goalPosition = goalPosition.transformBy(new Transform2d(-0.115*2, 0, new Rotation2d(Units.degreesToRadians(180))));
            } else {
                leftInstead = false;
            }

            goalPosition = AllianceFlipUtil.apply(goalPosition);

            Logger.recordOutput("AutoShoot/Goal Position", goalPosition);
        }

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

        ControllerOutput output = alignUtil.calculate(currentPose, goalPosition);

        drivetrain.setControl(drive.withVelocityX(-output.xVel).withVelocityY(-output.yVel).withRotationalRate(output.rotVel));

        if (Math.abs(currentPose.getX() - goalPosition.getX()) < 0.025
        && Math.abs(currentPose.getY() - goalPosition.getY()) < 0.025
        && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 5){
            setState(State.READY);
        }
        break;
        case READY:
            if (operator.getHID().getYButton()) {
                if (leftInstead) {
                    stateManager.state = StateManager.State.L3_LEFT;
                } else {
                    stateManager.state = StateManager.State.L3_RIGHT;
                }
            } else{
                if (leftInstead) {
                    stateManager.state = StateManager.State.L2_LEFT;
                } else {
                    stateManager.state = StateManager.State.L2_RIGHT;
                }
            }
            break;
    }
    }

    @Override
    public void end(boolean interrupted) {
        stateManager.state = StateManager.State.IDLE;
    }

    private void setState(State state){
        this.state = state;
    }
}