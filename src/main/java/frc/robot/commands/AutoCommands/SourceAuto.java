package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.subsystems.superstructure.StateManager.State;

public class SourceAuto extends Command{
    private final StateManager stateManager;

    public SourceAuto(StateManager stateManager) {
        this.stateManager = stateManager;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        stateManager.state = State.SOURCE_INTAKE;
    }

    @Override
    public void end(boolean interrupted) {
        stateManager.state = State.IDLE;
    }

    @Override
    public boolean isFinished() {
        return SmartDashboard.getBoolean("Deployer Ready", false);
    }
}
