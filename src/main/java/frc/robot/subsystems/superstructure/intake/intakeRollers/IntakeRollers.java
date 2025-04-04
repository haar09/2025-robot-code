package frc.robot.subsystems.superstructure.intake.intakeRollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakeRollers{
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static IntakeRollers create() {
        if (Robot.isReal()) {
            return new IntakeRollers(
                new RealIntakeRollers(IntakeConstants.kRollerMotorId, IntakeConstants.kRollerMotorReversed));
        } else {
            return new IntakeRollers(new NoIntakeRollers());
        }
    }

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Intake is disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Rollers", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
