package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.superstructure.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.superstructure.intake.intakeRollers.IntakeRollers;

public class Intake extends SubsystemBase{
    public final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;

    public Intake() {
        this.intakePivot = IntakePivot.create();
        this.intakeRollers = IntakeRollers.create();
    }

    public enum IntakeState {
        IDLE,
        FLOOR_INTAKE,
        DAYA,
        FEED,
        SHOOT,
        ALGAE,
        SOURCE,
        ELEVATOR
    }

    public IntakeState state = IntakeState.IDLE;

    @Override
    public void periodic() {
        intakePivot.periodic();
        intakeRollers.periodic();
        switch (state) {
            case IDLE:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.idleAngle.get()));
                intakeRollers.stop();
                break;
            case FLOOR_INTAKE:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.intakeAngle.get()));
                if (intakePivot.isAtDesiredAngle()) {
                    state = IntakeState.DAYA;
                }
                intakeRollers.setOutputPercentage(0.65);
                break;
            case DAYA:
                intakePivot.setVoltage(-0.4);
                intakeRollers.setOutputPercentage(0.65);
            case FEED:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.feedAngle.get())); 
                if (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(0.9);
                } else {
                    intakeRollers.setOutputPercentage(0);
                }
                break;
            case ALGAE:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.algaeAngle.get()));
                intakeRollers.setOutputPercentage(-0.6); 
                break;
            case SOURCE:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.elevatorAngle.get()));
                //intakeRollers.setOutputPercentage(-0.6); 
                break;
            case SHOOT:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.shootAngle.get()));
                if  (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(-0.8);
                } else {
                    intakeRollers.setOutputPercentage(0);
                }
                break;
            case ELEVATOR:
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.elevatorAngle.get()));
                intakeRollers.setOutputPercentage(0);
        }
        Logger.recordOutput("Intake/State", state.toString());
    }

    public boolean isAtDesiredAngle(){
        return intakePivot.isAtDesiredAngle();
    }

    public boolean elevatorClearance(){
        return intakePivot.getAngle().lte(Degrees.of(IntakeConstants.elevatorAngle.get()));
    }

    public Command setStateCommand(IntakeState state) {
        return runOnce(() -> this.state = state).withName("Intake "+state.toString());
    }

    public void setState(IntakeState state) {
        this.state = state;
    }
}
