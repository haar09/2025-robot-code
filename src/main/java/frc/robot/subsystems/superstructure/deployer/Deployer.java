package frc.robot.subsystems.superstructure.deployer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.deployer.deployerOmnis.DeployerOmnis;
import frc.robot.subsystems.superstructure.deployer.deployerRollers.DeployerRollers;

public class Deployer extends  SubsystemBase {
    private final DeployerOmnis deployerOmnis;
    private final DeployerRollers deployerRollers;
    public final DeployerBeamBreak deployerBeamBreak;

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
    //private boolean nomore = false;

    @Override
    public void periodic() {
        Logger.recordOutput("Deployer/State", state.name());
        deployerOmnis.periodic();
        deployerRollers.periodic();
        switch (state) {
            case IDLE:
                deployerOmnis.setOutputPercentage(0);
                deployerRollers.setOutputPercentage(0);
                //nomore = false;
                break;
            case CENTER: 
                deployerOmnis.setOutputPercentage(0.2);
                if (!deployerBeamBreak.left_value && !deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0);
                    break;
                }
                if (deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0);
                    state = DeployerState.IDLE;
                    break;
                }
                if (deployerBeamBreak.left_value && !deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(0.08);
                }
                if (!deployerBeamBreak.left_value && deployerBeamBreak.right_value) {
                    deployerRollers.setOutputPercentage(-0.08);
                }
                break;
            case SHOOT_RIGHT:
                deployerOmnis.stop();
                /*if (deployerBeamBreak.right_value && !nomore) {
                    deployerRollers.setOutputPercentage(-0.1);
                } else {
                    nomore = true;*/
                    deployerRollers.setOutputPercentage(0.20);
                //}
                break;
            case SHOOT_LEFT:
                deployerOmnis.stop();
                /*if (deployerBeamBreak.left_value && !nomore) {
                    deployerRollers.setOutputPercentage(0.1);
                } else {
                    nomore = true;*/
                    deployerRollers.setOutputPercentage(-0.20);
                //}
                break;
        }
    }

    public void setState(DeployerState state) {
        this.state = state;
    }
}
