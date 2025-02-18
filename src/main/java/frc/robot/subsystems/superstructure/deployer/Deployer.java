package frc.robot.subsystems.superstructure.deployer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.deployer.deployerOmnis.DeployerOmnis;
import frc.robot.subsystems.superstructure.deployer.deployerRollers.DeployerRollers;
import frc.robot.util.AllianceFlipUtil;

public class Deployer extends  SubsystemBase {
    private final DeployerOmnis deployerOmnis;
    private final DeployerRollers deployerRollers;
    private final DeployerBeamBreak deployerBeamBreak;

    private final CommandSwerveDrivetrain drivetrain;

    private double startTime = 0;

    public Deployer(CommandSwerveDrivetrain drivetrain){
        this.deployerOmnis = DeployerOmnis.create();
        this.deployerRollers = DeployerRollers.create();
        this.deployerBeamBreak = new DeployerBeamBreak();
        this.drivetrain = drivetrain;
    }

    public enum DeployerState {
        IDLE,
        CENTER,
        SHOOT_RIGHT,
        SHOOT_LEFT,
        AUTO_SHOOT
    }

    public DeployerState state = DeployerState.IDLE;

    @Override
    public void periodic() {
        deployerOmnis.periodic();
        deployerRollers.periodic();
        calculateReefSide();
        switch (state) {
            case IDLE:
                deployerOmnis.stop();
                deployerRollers.stop();
                break;
            case CENTER: 
                if (!deployerBeamBreak.left_value || !deployerBeamBreak.right_value) {
                    deployerOmnis.setOutputPercentage(0.65);
                    deployerRollers.setOutputPercentage(0);
                    break;
                }
                if (deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    state = DeployerState.IDLE;
                    break;
                }
                if (deployerBeamBreak.left_value && !deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(-0.3);
                }
                if (!deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0.3);
                }
                break;
            case SHOOT_RIGHT:
                if (Timer.getFPGATimestamp() - startTime < 0.1) {
                    deployerRollers.setOutputPercentage(-0.2);
                } else {
                    deployerRollers.setOutputPercentage(0.65);
                }
                break;
            case SHOOT_LEFT:
                if (Timer.getFPGATimestamp() - startTime < 0.1) {
                    deployerRollers.setOutputPercentage(0.2);
                } else {
                    deployerRollers.setOutputPercentage(0.65);
                }
                break;
            case AUTO_SHOOT:
                if (GlobalVariables.getInstance().alliance == Alliance.Blue) {
                } else {
                    setState(DeployerState.SHOOT_RIGHT);
                }
                break;
        }
    }

    private double reefAngle = 0;
    private double targetAngle = 0;

    public void calculateReefSide() {
        //drivetrain.getState().Pose.nearest(FieldConstants.Reef.centerFaces);
    }

    public void setState(DeployerState state) {
        this.state = state;
        startTime = Timer.getFPGATimestamp();
    }
}
