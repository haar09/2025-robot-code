package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.util.AlignUtil;
import frc.robot.util.AllianceFlipUtil;

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

    private boolean isMovedforL3 = false;

    @Override
    public void initialize() {
        isMovedforL3 = false;
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

            if (operator.getHID().getYButton()){
                goalPosition = goalPosition.transformBy(new Transform2d(ElevatorConstants.kL3Offset, 0 , new Rotation2d(Units.degreesToRadians(0))));
                isMovedforL3 = true;
            }

            goalPosition = AllianceFlipUtil.apply(goalPosition);
        }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private enum State{
        TRAVELLING,
        READY
    }

    private State state = State.TRAVELLING;

    @Override
    public void execute() {
        Logger.recordOutput("AutoShoot/L3 Offset", isMovedforL3);
        if (operator.getHID().getYButton() && !isMovedforL3){
            goalPosition = goalPosition.transformBy(new Transform2d(ElevatorConstants.kL3Offset, 0 , new Rotation2d(Units.degreesToRadians(0))));
            state = State.TRAVELLING;
            isMovedforL3 = true;
        } else if (!operator.getHID().getYButton() && isMovedforL3){
            goalPosition = goalPosition.transformBy(new Transform2d(-ElevatorConstants.kL3Offset, 0 , new Rotation2d(Units.degreesToRadians(0))));
            state = State.TRAVELLING;
            isMovedforL3 = false;
        }

        switch (state){
            case TRAVELLING:
        Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition, leftInstead);
    
            drivetrain.setControl(drive.withVelocityX(output.vxMetersPerSecond).withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    
        Logger.recordOutput("AutoShoot/Left Distance", currentPose.minus(goalPosition).getTranslation().getNorm());
        if (currentPose.minus(goalPosition).getTranslation().getNorm() < 0.02
        && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 2){
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
        GlobalVariables.getInstance().alignStatus = 0;
    }

    private void setState(State state){
        this.state = state;
    }
}