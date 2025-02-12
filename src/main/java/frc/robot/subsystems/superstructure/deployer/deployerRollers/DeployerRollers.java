package frc.robot.subsystems.superstructure.deployer.deployerRollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;

public class DeployerRollers{
    private final DeployerRollersIO io;
    private final DeployerRollersIOInputsAutoLogged inputs = new DeployerRollersIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static DeployerRollers create() {
        if (Robot.isReal()) {
            return new DeployerRollers(
                new RealDeployerRollers());
        } else {
            return new DeployerRollers(new NoDeployerRollers());
        }
    }

    public DeployerRollers(DeployerRollersIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Deployer Rollers are disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deployer/Rollers", inputs);
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
