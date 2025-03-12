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
        SOURCE,
        ELEVATOR,
        BEFORE_FEED
    }

    public IntakeState state = IntakeState.IDLE;

    private boolean hasSeen = false;

    @Override
    public void periodic() {
        intakePivot.periodic();
        intakeRollers.periodic();
        switch (state) {
            case IDLE:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.idleAngle.get()));
                intakeRollers.stop();
                if (intakePivot.isAtDesiredAngle()) {
                    hasSeen = false;
                }
                break;
            case FLOOR_INITIAL:
                intakePivot.setCoast();
                if (intakeBeamBreak.lower_value || hasSeen) {
                    state = IntakeState.FLOOR_INTAKE;
                    hasSeen = true;
                    break;
                }
                if (intakeBeamBreak.upper_value) {
                    state = IntakeState.IDLE;
                    break;
                }
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.initialAngle.get()));
                intakeRollers.setOutputPercentage(-0.3,-0.15);   
                break;
            case FLOOR_INTAKE:
                if (intakeBeamBreak.upper_value) {
                    state = IntakeState.BEFORE_FEED;
                    hasSeen = false;
                    break;
                }
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.intakeAngle.get()));
                intakeRollers.setOutputPercentage(-0.09, -0.08);
                break;
            case BEFORE_FEED:
                intakeRollers.setOutputPercentage(0, 0);
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.idleAngle.get()));
                break;
            case FEED:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.feedAngle.get())); 
                if (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(-0.3, 0);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ALGAE:
                intakePivot.setCoast();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.algaeAngle.get()));
                intakeRollers.setOutputPercentage(0, 0.6); 
                break;
            case SOURCE:
                intakePivot.setCoast();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.shootAngle.get()));
                intakeRollers.setOutputPercentage(0.6, 0); 
                break;
            case SHOOT:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.shootAngle.get()));
                if  (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(0.8, 0.8);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ELEVATOR:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(Degrees.of(IntakeConstants.elevatorAngle.get()));
                intakeRollers.setOutputPercentage(0, 0);
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

    public void setStateifNotBusy(IntakeState state) {
        if (intakeBeamBreak.upper_value) {
            return;
        }
        if (this.state == IntakeState.IDLE) {
            this.state = state;
        }
    }

    public void setState(IntakeState state) {
        this.state = state;
    }
}
