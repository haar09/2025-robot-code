package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;

public class Intake extends SubsystemBase{
    private final IntakePivot intakePivot;
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
        SHOOT
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
                intakeRollers.setOutputPercentage(0.4, 0.4);   
                break;
            case FLOOR_INTAKE:
                if (intakeBeamBreak.upper_value) {
                    state = IntakeState.FEED;
                    break;
                }
                intakePivot.setDesiredAngle(IntakeConstants.intakeAngle);
                intakeRollers.setOutputPercentage(0.4, 0.4);
                break;
            case FEED:
                intakePivot.setDesiredAngle(IntakeConstants.feedAngle); 
                if (Math.abs(intakePivot.getAngle().in(Degrees)-IntakeConstants.feedAngle.in(Radians)) < 0.3) {
                    intakeRollers.setOutputPercentage(0.4, 0.4);
                } else {
                    intakeRollers.setOutputPercentage(0, 0);
                }
                break;
            case SHOOT:
                intakePivot.setDesiredAngle(IntakeConstants.shootAngle);
                intakeRollers.setOutputPercentage(-0.9, -0.9);
                break;
        }
        SmartDashboard.putString("Intake State", state.toString());
    }

    public Command setState(IntakeState state) {
        return runOnce(() -> this.state = state).withName("Intake "+state.toString());
    }
}
