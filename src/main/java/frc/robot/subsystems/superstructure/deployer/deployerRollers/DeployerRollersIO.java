package frc.robot.subsystems.superstructure.deployer.deployerRollers;

import org.littletonrobotics.junction.AutoLog;

public interface DeployerRollersIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(DeployerRollersIOInputs inputs) {}

    @AutoLog
    class DeployerRollersIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
}