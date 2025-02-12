package frc.robot.subsystems.superstructure.deployer.deployerOmnis;

import org.littletonrobotics.junction.AutoLog;

public interface DeployerOmnisIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(DeployerOmnisIOInputs inputs) {}

    @AutoLog
    class DeployerOmnisIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
}