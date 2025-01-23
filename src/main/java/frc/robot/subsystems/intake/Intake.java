package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;

public class Intake extends SubsystemBase{
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    
    public Intake(IntakePivot intakePivot, IntakeRollers intakeRollers) {
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
    }

    public enum IntakeState {
        IDLE,
        FLOOR_INITIAL,
        FLOOR_INTAKE,
        FEED,
        SHOOT
    }

    public IntakeState state = IntakeState.IDLE;

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                intakePivot.setDesiredAngle(IntakeConstants.idleAngle);
                intakeRollers.stop();
                break;
            case FLOOR_INITIAL:
                intakePivot.setDesiredAngle(IntakeConstants.initialAngle);
                intakeRollers.setOutputPercentage(0.4, 0.4);
                break;
            case FLOOR_INTAKE:
                intakePivot.setDesiredAngle(IntakeConstants.intakeAngle);
                intakeRollers.setOutputPercentage(0.4, 0.4);
                break;
            case FEED:
                intakePivot.setDesiredAngle(IntakeConstants.feedAngle); 
                intakeRollers.setOutputPercentage(0.4, 0.4);
                break;
            case SHOOT:
                intakePivot.setDesiredAngle(IntakeConstants.shootAngle);
                intakeRollers.setOutputPercentage(-0.9, -0.9);
                break;
        }
    }

    public Command setState(IntakeState state) {
        return runOnce(() -> this.state = state).withName("Intake "+state.toString());
    }
}
