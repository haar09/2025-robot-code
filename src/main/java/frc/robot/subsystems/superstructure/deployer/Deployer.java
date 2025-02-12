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

}
