package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.util.AlignUtil;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NetworkTablesAgent;

public class AutoBranchandShootL1 extends Command{
    private final NetworkTablesAgent networkTablesAgent;
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;
    private final AlignUtil alignUtil = new AlignUtil();

    private String reefBranch;

    private Pose2d goalPosition;

    public AutoBranchandShootL1(NetworkTablesAgent networkTablesAgent, CommandSwerveDrivetrain drivetrain, StateManager stateManager) {
        this.networkTablesAgent = networkTablesAgent;
        this.drivetrain = drivetrain;
        this.stateManager = stateManager;
    }

    @Override
    public void initialize() {
        state = State.TRAVELLING;

        reefBranch = networkTablesAgent.buttonValue.get();

        Pose2d currentPose = drivetrain.getState().Pose;

        alignUtil.resetControllers(currentPose, drivetrain.getState().Speeds);

            if(reefBranch.contentEquals("A")){
                goalPosition = Reef.scoringPositions2d.get(1).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("B")){
                goalPosition = Reef.scoringPositions2d.get(0).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("C")){
                goalPosition = Reef.scoringPositions2d.get(11).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("D")){
                goalPosition = Reef.scoringPositions2d.get(10).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("E")){
                goalPosition = Reef.scoringPositions2d.get(9).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("F")){
                goalPosition = Reef.scoringPositions2d.get(8).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("G")){
                goalPosition = Reef.scoringPositions2d.get(7).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("H")){
                goalPosition = Reef.scoringPositions2d.get(6).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("I")){
                goalPosition = Reef.scoringPositions2d.get(5).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("J")){
                goalPosition = Reef.scoringPositions2d.get(4).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("K")){
                goalPosition = Reef.scoringPositions2d.get(3).get(ReefLevel.L1);
            } else if (reefBranch.contentEquals("L")){
                goalPosition = Reef.scoringPositions2d.get(2).get(ReefLevel.L1);
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
        switch (state) {
            case TRAVELLING:
            Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition);
    
            drivetrain.setControl(drive.withVelocityX(output.vxMetersPerSecond).withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    
            if (Math.abs(currentPose.getX() - goalPosition.getX()) < 0.025
            && Math.abs(currentPose.getY() - goalPosition.getY()) < 0.025
            && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 3){
                setState(State.READY);
            }
                break;
        
            case READY:
            stateManager.state = StateManager.State.L1;
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