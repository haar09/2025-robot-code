package frc.robot.subsystems.intake.intakeRollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollers{
    private final IntakeRollersIO ioLeft;
    private final IntakeRollersIO ioRight;
    private final IntakeRollersIOInputsAutoLogged inputsLeft = new IntakeRollersIOInputsAutoLogged();
    private final IntakeRollersIOInputsAutoLogged inputsRight = new IntakeRollersIOInputsAutoLogged();
    private final Alert disconnectedAlertLeft, disconnectedAlertRight;

    public static IntakeRollers create() {
        if (Robot.isReal()) {
            return new IntakeRollers(
                new RealIntakeRollers(IntakeConstants.kRollerLeftMotorId, IntakeConstants.kRollerLeftMotorReversed),
                new RealIntakeRollers(IntakeConstants.kRollerRightMotorId, IntakeConstants.kRollerRightMotorReversed));
        } else {
            return new IntakeRollers(new NoIntakeRollers(), new NoIntakeRollers());
        }
    }

    public IntakeRollers(IntakeRollersIO ioLeft, IntakeRollersIO ioRight) {
        this.ioLeft = ioLeft;
        this.ioRight = ioRight;
        this.disconnectedAlertLeft = new Alert("Intake Left is disconnected.", AlertType.kWarning);
        this.disconnectedAlertRight = new Alert("Intake Right is disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        ioLeft.updateInputs(inputsLeft);
        ioRight.updateInputs(inputsRight);
        Logger.processInputs("Intake/Rollers/Left", inputsLeft);
        Logger.processInputs("Intake/Rollers/Right", inputsRight);
        disconnectedAlertLeft.set(!inputsLeft.motorConnected);
        disconnectedAlertRight.set(!inputsRight.motorConnected);
    }

    public void setOutputPercentage(double percentageLeft, double percentageRight) {
        ioLeft.setOutputPercentage(percentageLeft);
        ioRight.setOutputPercentage(percentageRight);
    }

    public void stop(){
        ioLeft.setOutputPercentage(0);
        ioRight.setOutputPercentage(0);
    }
}
