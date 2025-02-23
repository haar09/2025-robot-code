package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        intakePivot.resetEncoders();

        new Trigger(() -> intakePivot.getAngle().in(Degrees) > 40 && intakePivot.getVelocity() < 0.3).toggleOnTrue(runOnce(() -> intakePivot.resetEncoders()));
    }

    public enum IntakeState {
        IDLE,
        FLOOR_INITIAL,
        FLOOR_INTAKE,
        FEED,
        SHOOT,
        ALGAE,
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
                intakePivot.setDesiredAngle(IntakeConstants.idleAngle);
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
                intakePivot.setDesiredAngle(IntakeConstants.initialAngle);
                intakeRollers.setOutputPercentage(-0.4,-0.2);   
                break;
            case FLOOR_INTAKE:
                if (intakeBeamBreak.upper_value) {
                    state = IntakeState.BEFORE_FEED;
                    hasSeen = false;
                    break;
                }
                intakePivot.setDesiredAngle(IntakeConstants.intakeAngle);
                intakeRollers.setOutputPercentage(-0.17, -0.13);
                /*if (intakePivot.isAtDesiredAngle() && Timer.getFPGATimestamp() - timer > 0.3) {
                    state = IntakeState.IDLE;
                    break;
                }*/
                break;
            case BEFORE_FEED:
                intakeRollers.setOutputPercentage(0, 0);
                intakePivot.setBrake();
                intakePivot.setSlowAngle(IntakeConstants.idleAngle);
                break;
            case FEED:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(IntakeConstants.feedAngle); 
                if (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(-0.3, 0);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ALGAE:
                intakePivot.setCoast();
                intakePivot.setDesiredAngle(IntakeConstants.algaeAngle);
                intakeRollers.setOutputPercentage(0, 0.6); 
                break;
            case SHOOT:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(IntakeConstants.shootAngle);
                if  (intakePivot.isAtDesiredAngle()) {
                    intakeRollers.setOutputPercentage(0.9, 0.9);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case ELEVATOR:
                intakePivot.setBrake();
                intakePivot.setDesiredAngle(IntakeConstants.elevatorAngle);
                intakeRollers.setOutputPercentage(0, 0);
        }
        Logger.recordOutput("Intake/State", state.toString());
    }

    public boolean isAtDesiredAngle(){
        return intakePivot.isAtDesiredAngle();
    }

    public boolean elevatorClearance(){
        return intakePivot.getAngle().lte(IntakeConstants.elevatorAngle);
    }

    public Command setStatecCommand(IntakeState state) {
        return runOnce(() -> this.state = state).withName("Intake "+state.toString());
    }

    public void setStateifNotBusy(IntakeState state) {
        if (intakeBeamBreak.upper_value) {
            return;
        }
        if (this.state == IntakeState.IDLE) {
            intakePivot.resetEncoders();
            this.state = state;
        }
    }

    public void setState(IntakeState state) {
        this.state = state;
    }
}
