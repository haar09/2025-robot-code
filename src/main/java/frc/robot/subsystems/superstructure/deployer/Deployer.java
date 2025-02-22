package frc.robot.subsystems.superstructure.deployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.deployer.deployerOmnis.DeployerOmnis;
import frc.robot.subsystems.superstructure.deployer.deployerRollers.DeployerRollers;

public class Deployer extends  SubsystemBase {
    private final DeployerOmnis deployerOmnis;
    private final DeployerRollers deployerRollers;
    private final DeployerBeamBreak deployerBeamBreak;

    public Deployer(){
        this.deployerOmnis = DeployerOmnis.create();
        this.deployerRollers = DeployerRollers.create();
        this.deployerBeamBreak = new DeployerBeamBreak();
    }

    public enum DeployerState {
        IDLE,
        CENTER,
        SHOOT_RIGHT,
        SHOOT_LEFT
    }

    public DeployerState state = DeployerState.IDLE;

    @Override
    public void periodic() {
        deployerOmnis.periodic();
        deployerRollers.periodic();
        switch (state) {
            case IDLE:
                deployerOmnis.stop();
                deployerRollers.stop();
                break;
            case CENTER: 
                deployerOmnis.setOutputPercentage(0.2);
                if (!deployerBeamBreak.left_value && !deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0);
                    break;
                }
                if (deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    state = DeployerState.IDLE;
                    break;
                }
                if (deployerBeamBreak.left_value && !deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0.1);
                }
                if (!deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(-0.1);
                }
                break;
            case SHOOT_RIGHT:
                deployerOmnis.stop();
                if (deployerBeamBreak.left_value) {
                    deployerRollers.setOutputPercentage(0.1);
                } else {
                    deployerRollers.setOutputPercentage(-0.9);
                }
                break;
            case SHOOT_LEFT:
                deployerOmnis.stop();
                if (deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(-0.1);
                } else {
                    deployerRollers.setOutputPercentage(0.9);
                }
                break;
        }
    }

    public void setState(DeployerState state) {
        this.state = state;
    }
}
