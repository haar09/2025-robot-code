package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.superstructure.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.superstructure.intake.intakeRollers.IntakeRollers;

public class Intake extends SubsystemBase{
    public final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final IntakeBeamBreak intakeBeamBreak;

    public Intake() {
        this.intakePivot = IntakePivot.create();
        this.intakeRollers = IntakeRollers.create();
        this.intakeBeamBreak = new IntakeBeamBreak();
    }

    public enum IntakeState {
        IDLE,
        FLOOR_INITIAL,
        FLOOR_INTAKE,
        FEED,
        SHOOT,
        ALGAE,
        ELEVATOR
    }

    public IntakeState state = IntakeState.IDLE;

    @Override
    public void periodic() {
        intakePivot.periodic();
        intakeRollers.periodic();
        switch (state) {
            case IDLE:
                intakePivot.setDesiredAngle(IntakeConstants.idleAngle);
                intakeRollers.stop();
                break;
            case FLOOR_INITIAL:
                if (intakeBeamBreak.lower_value || intakeBeamBreak.upper_value) {
                    state = IntakeState.FLOOR_INTAKE;
                    break;
                }
                intakePivot.setDesiredAngle(IntakeConstants.initialAngle);
                intakeRollers.setOutputPercentage(-0.4,-0.4);   
                break;
            case FLOOR_INTAKE:
                if (intakeBeamBreak.upper_value) {
                    state = IntakeState.IDLE;
                    break;
                }
                intakePivot.setDesiredAngle(IntakeConstants.intakeAngle);
                intakeRollers.setOutputPercentage(-0.4, -0.4);
                break;
            case FEED:
                intakePivot.setDesiredAngle(IntakeConstants.feedAngle); 
                if (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(-0.4, -0.4);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ALGAE:
                intakePivot.setDesiredAngle(IntakeConstants.algaeAngle);
                intakeRollers.setOutputPercentage(0, 0.6); 
                break;
            case SHOOT:
                intakePivot.setDesiredAngle(IntakeConstants.shootAngle);
                if  (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(0.9, 0.9);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ELEVATOR:
                intakePivot.setDesiredAngle(IntakeConstants.elevatorAngle);
                intakeRollers.setOutputPercentage(0, 0);
        }
        Logger.recordOutput("Intake/State", state.toString());
    }

    public boolean isAtDesiredAngle(){
        return intakePivot.isAtDesiredAngle();
    }

    public Command setState(IntakeState state) {
        return runOnce(() -> this.state = state).withName("Intake "+state.toString());
    }
}
