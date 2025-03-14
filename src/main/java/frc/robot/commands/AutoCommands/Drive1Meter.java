package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AlignUtil;

public class Drive1Meter extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final AlignUtil alignUtil = new AlignUtil();
    private Pose2d goalPosition;

    public Drive1Meter(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;
                alignUtil.resetControllers(currentPose, drivetrain.getState().Speeds);

        goalPosition = currentPose.transformBy(new Transform2d(0, 0, Rotation2d.kCCW_90deg));
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(0).withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void execute() {
        SmartDashboard.putBoolean("aligned", false);
        Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition);
    
            drivetrain.setControl(drive.withVelocityX(output.vxMetersPerSecond).withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    }
}
