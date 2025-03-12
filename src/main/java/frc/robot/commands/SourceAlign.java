package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.util.AlignUtil;
import frc.robot.util.AllianceFlipUtil;

public class SourceAlign extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;
    private final AlignUtil alignUtil = new AlignUtil();

    private Pose2d goalPosition;

    public SourceAlign(CommandSwerveDrivetrain drivetrain, StateManager stateManager) {
        this.drivetrain = drivetrain;
        this.stateManager = stateManager;
    }

    @Override
    public void initialize() {
        state = State.TRAVELLING;
        Pose2d currentPose = drivetrain.getState().Pose;

        alignUtil.resetControllers(currentPose, drivetrain.getState().Speeds);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            
        }       

            goalPosition = AllianceFlipUtil.apply(goalPosition);

            Logger.recordOutput("AutoShoot/Goal Position", goalPosition);
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
        switch (state){
            case TRAVELLING:
        Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition);
    
            drivetrain.setControl(drive.withVelocityX(output.vxMetersPerSecond).withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    
        if (Math.abs(currentPose.getX() - goalPosition.getX()) < 0.02
        && Math.abs(currentPose.getY() - goalPosition.getY()) < 0.02
        && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 3){
            setState(State.READY);
        }
        break;
        case READY:
            stateManager.state = StateManager.State.SOURCE_INTAKE;
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