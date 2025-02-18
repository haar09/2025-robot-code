package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.deployer.Deployer;
import frc.robot.subsystems.superstructure.deployer.Deployer.DeployerState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.Intake.IntakeState;

public class StateManager extends SubsystemBase{
    private final Deployer deployer;
    private final Intake intake;
    private final Elevator elevator;

    public StateManager(Deployer deployer, Intake intake, Elevator elevator){
        this.deployer = deployer;
        this.intake = intake;
        this.elevator = elevator;
    }

    public enum State {
        IDLE,
        CORAL_INTAKE,
        ALGAE_INTAKE,
        FEED,
        L1,
        L2,
        L3
    }
    public State state = State.IDLE;

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                intake.state = IntakeState.IDLE;
                deployer.state = DeployerState.IDLE;
                elevator.setPosition(ElevatorConstants.IDLE);
                break;
            case CORAL_INTAKE:
                intake.state = IntakeState.FLOOR_INITIAL;
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.IDLE);
                break;
            case ALGAE_INTAKE:
                intake.state = IntakeState.ALGAE;
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.IDLE);
                break;
            case FEED:
                intake.state = IntakeState.FEED;
                deployer.setState(DeployerState.CENTER);
                elevator.setPosition(ElevatorConstants.INTAKE_HEIGHT);
                break;
            case L1:
                intake.state = IntakeState.SHOOT;
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.IDLE);
                break;
            case L2:
                intake.state = IntakeState.ALGAE;
                deployer.state = DeployerState.IDLE;
                elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT);
                if (elevator.isAtSetpoint()) {
                    intake.state = IntakeState.IDLE;
                    deployer.state = DeployerState.SHOOT_RIGHT;
                }
                break;
            case L3:
                intake.state = IntakeState.SHOOT;
                deployer.state = DeployerState.IDLE;
                elevator.setPosition(ElevatorConstants.CORAL_L3_HEIGHT);
                if (elevator.isAtSetpoint()) {
                    intake.state = IntakeState.IDLE;
                    deployer.state = DeployerState.SHOOT_RIGHT;
                }
            break;
        }
    }
}
