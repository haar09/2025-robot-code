package frc.robot.subsystems.superstructure.deployer.deployerOmnis;

import edu.wpi.first.wpilibj.RobotController;

public class NoDeployerOmnis implements DeployerOmnisIO {
    private double appliedVoltage = 0;

    public NoDeployerOmnis() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        appliedVoltage = percentage * RobotController.getBatteryVoltage();
    }

    @Override
    public void stop(){
        appliedVoltage = 0;
    }

    @Override
    public void updateInputs(DeployerOmnisIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
    }
}