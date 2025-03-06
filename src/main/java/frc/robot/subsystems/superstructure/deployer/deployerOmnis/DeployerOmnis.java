package frc.robot.subsystems.superstructure.deployer.deployerOmnis;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;

public class DeployerOmnis{
    private final DeployerOmnisIO io;
    private final DeployerOmnisIOInputsAutoLogged inputs = new DeployerOmnisIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static DeployerOmnis create() {
        if (Robot.isReal()) {
            return new DeployerOmnis(
                new RealDeployerOmnis());
        } else {
            return new DeployerOmnis(new NoDeployerOmnis());
        }
    }

    public DeployerOmnis(DeployerOmnisIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Deployer Omnis are disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deployer/Omnis", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
        io.setOutputPercentage(0);
    }
}
