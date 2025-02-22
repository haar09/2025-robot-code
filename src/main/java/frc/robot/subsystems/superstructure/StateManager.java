package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.deployer.Deployer;
import frc.robot.subsystems.superstructure.deployer.Deployer.DeployerState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.Intake.IntakeState;
import frc.robot.util.AllianceFlipUtil;

public class StateManager extends SubsystemBase{
    private final Deployer deployer;
    private final Intake intake;
    private final Elevator elevator;
    private final CommandSwerveDrivetrain drivetrain;

    public StateManager(Deployer deployer, Intake intake, Elevator elevator, CommandSwerveDrivetrain drivetrain){
        this.deployer = deployer;
        this.intake = intake;
        this.elevator = elevator;
        this.drivetrain = drivetrain;
        SmartDashboard.putData("Field2", field2d);
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
            case L2:
                deployer.setState(DeployerState.IDLE); 
                intake.setState(IntakeState.ELEVATOR);
                if (intake.elevatorClearance()){
                elevator.setPosition(Centimeters.of(SmartDashboard.getNumber("asansoryukseklik", 0)));
                deployer.setState(DeployerState.CENTER);
                if (elevator.isAtSetpoint()) {
                    deployer.setState(calculateDeployerSide());;
                }
            }
                break;
            case L3:
                intake.setState(IntakeState.ELEVATOR);
                deployer.setState(DeployerState.IDLE);
                elevator.setPosition(ElevatorConstants.CORAL_L3_HEIGHT);
                if (elevator.isAtSetpoint()) {
                    intake.setState(IntakeState.IDLE);
                    deployer.state = calculateDeployerSide();
                }
            break;
        }
    }

    private final Field2d field2d = new Field2d();

    public DeployerState calculateDeployerSide() {
        List<Pose2d> flippedCenterFaces = Arrays.stream(FieldConstants.Reef.centerFaces)
            .map(AllianceFlipUtil::apply)
            .collect(Collectors.toList());

        if ((drivetrain.getState().Pose.getRotation().getDegrees() 
        - drivetrain.getState().Pose.nearest(flippedCenterFaces).getRotation().getDegrees() 
        + 360) % 360 > 180 || SmartDashboard.getString("Arduino/Branch", "Z") == "O"){
            return DeployerState.SHOOT_RIGHT;
        } else {
            return DeployerState.SHOOT_LEFT;
        }
    }
    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.IDLE).withName("StateManager "+state.toString());
    }
}
