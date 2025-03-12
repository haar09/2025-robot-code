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

        goalPosition = currentPose.transformBy(new Transform2d(1, 0, new Rotation2d(0)));
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void execute() {
        SmartDashboard.putBoolean("aligned", false);
        Pose2d  currentPose = drivetrain.getState().Pose;

            ChassisSpeeds output = alignUtil.calculate(currentPose, goalPosition);
    
            drivetrain.setControl(drive.withVelocityX(output.vxMetersPerSecond).withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    
            if (Math.abs(currentPose.getX() - goalPosition.getX()) < 0.02
            && Math.abs(currentPose.getY() - goalPosition.getY()) < 0.02
            && Math.abs(currentPose.getRotation().minus(goalPosition.getRotation()).getDegrees()) < 3){
                SmartDashboard.putBoolean("aligned", true);
            }
    }
}
