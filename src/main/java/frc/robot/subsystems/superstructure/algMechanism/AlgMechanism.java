package frc.robot.subsystems.superstructure.algMechanism;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.AlgMechanismConstants;

public class AlgMechanism extends SubsystemBase{
    private final AlgMechanismIO io;
    private final AlgMechanismIOInputsAutoLogged inputs = new AlgMechanismIOInputsAutoLogged();
    private final Alert disconnectedAlertRack/*, disconnectedAlertRoller*/;

    public static AlgMechanism create() {
        if (Robot.isReal()){
            return new AlgMechanism(new RealAlgMechanism(AlgMechanismConstants.kRackMotorId/*, AlgMechanismConstants.kRollerMotorId*/));
        } else {
            return new AlgMechanism(new NoAlgMechanism());
        }
    }

    public AlgMechanism(AlgMechanismIO io) {
        this.io = io;
        this.disconnectedAlertRack = new Alert("Alg Rack is disconnected.", Alert.AlertType.kWarning);
        //this.disconnectedAlertRoller = new Alert("Alg Roller is disconnected.", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgMechansim", inputs);
        disconnectedAlertRack.set(!inputs.rackMotorConnected);
        //disconnectedAlertRoller.set(!inputs.rollerMotorConnected);
    }

    public void setDesiredExtension(double extension) {
        io.setDesiredExtension(extension);
    }
    /*public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }*/
}
