package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        L2_RIGHT,
        L2_LEFT,
        L3_RIGHT,
        L3_LEFT,
        TEST
    }
    public State state = State.IDLE;

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
            deployer.setState(DeployerState.IDLE);
            elevator.setPosition(ElevatorConstants.IDLE);
                if (elevator.isAtSetpoint()) {
                    intake.setState(IntakeState.IDLE);
                }
                break;
            case CORAL_INTAKE:
                intake.setStateifNotBusy(IntakeState.FLOOR_INITIAL);
                deployer.setState(DeployerState.IDLE);
                if (intake.elevatorClearance()){
                    elevator.setPosition(ElevatorConstants.IDLE);
                }
                break;
            case ALGAE_INTAKE:
                intake.setState(IntakeState.ALGAE);
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.IDLE);
                break;
            case FEED:
                deployer.setState(DeployerState.CENTER);
                elevator.setPosition(ElevatorConstants.INTAKE_HEIGHT);
                if (elevator.isAtSetpoint()) {
                    intake.setState(IntakeState.FEED);
                }
                break;
            case L1:
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.IDLE);
                intake.setState(IntakeState.SHOOT);
                break;
            case L2_RIGHT:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT);
                deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(DeployerState.SHOOT_RIGHT);
                }
            }
                break;
            case L2_LEFT:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT);
                deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(DeployerState.SHOOT_LEFT);
                }
            }
                break;
            case L3_RIGHT:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT);
                deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(DeployerState.SHOOT_RIGHT);
                }
            }
            break;
            case L3_LEFT:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT);
                deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(DeployerState.SHOOT_LEFT);
                }
            }
            break;
            case TEST:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                    elevator.setPosition(Centimeters.of(SmartDashboard.getNumber("asansoryukseklik", 0))); 
                    deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(DeployerState.SHOOT_RIGHT);
                }
                }
                break;   
        }
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.IDLE).withName("StateManager "+state.toString());
    }
}
