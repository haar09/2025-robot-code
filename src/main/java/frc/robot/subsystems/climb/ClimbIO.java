package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    public void setOutputPercentage(double percentage);

    public void stop();

    default void updateInputs(ClimbIOInputs inputs) {}
    @AutoLog
    class ClimbIOInputs {
        public boolean climbMotorMotorConnected = true;
    
        public double climbMotorVelocityRps = 0.0;
        public double climbMotorAppliedVolts = 0.0;
        public double climbMotorSupplyCurrentAmps = 0.0;
        public double climbMotorTempCelsius = 0.0;
    
}
}