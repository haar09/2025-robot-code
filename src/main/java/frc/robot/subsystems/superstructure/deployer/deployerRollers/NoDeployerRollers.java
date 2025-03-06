package frc.robot.subsystems.superstructure.deployer.deployerRollers;

import edu.wpi.first.wpilibj.RobotController;

public class NoDeployerRollers implements DeployerRollersIO {
    private double appliedVoltage = 0;

    public NoDeployerRollers() {
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
    public void updateInputs(DeployerRollersIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
    }
}