package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.subsystems.superstructure.StateManager.State;
import frc.robot.subsystems.superstructure.deployer.Deployer.DeployerState;

public class L2Auto extends Command{
    private final StateManager stateManager;
    private final boolean leftside;

        public L2Auto(boolean leftside, StateManager stateManager) {
        this.stateManager = stateManager;
        this.leftside = leftside;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (leftside){
            stateManager.state = State.L2_LEFT;
        } else {
        stateManager.state = State.L2_RIGHT;
        }
    }

    @Override
    public void end(boolean interrupted) {
        stateManager.state = State.IDLE;
        stateManager.deployer.setState(DeployerState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return !stateManager.deployer.deployerBeamBreak.left_value && !stateManager.deployer.deployerBeamBreak.right_value;
    }
}
