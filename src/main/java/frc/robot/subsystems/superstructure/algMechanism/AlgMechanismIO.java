package frc.robot.subsystems.superstructure.algMechanism;

import org.littletonrobotics.junction.AutoLog;

public interface AlgMechanismIO {
    public void setDesiredExtension(double extension);
    public void setOutputPercentage(double percentage);

    default void updateInputs(AlgMechanismIOInputs inputs) {}

    @AutoLog
    class AlgMechanismIOInputs {
        public boolean rackMotorConnected = true;
        public boolean rollerMotorConnected = true;

        public double rackPositionRotations = 0;
        public double rackVelocityRPM = 0;
        public double rackAppliedVolts = 0;
        public double rackCurrentAmps = 0;
        public double rackTemperatureCelcius = 0;
        
        public double rollerPositionRotations = 0;
        public double rollerVelocityRPM = 0;
        public double rollerAppliedVolts = 0;
        public double rollerCurrentAmps = 0;
        public double rollerTemperatureCelcius = 0;
    }
}
