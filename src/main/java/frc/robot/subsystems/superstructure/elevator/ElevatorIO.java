package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    public void setPosition(double rotations);
    public void stop();

    default void updateInputs(ElevatorIOInputs inputs) {}

    @AutoLog
    class ElevatorIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;

        public double leftPositionRotations = 0.0;
        public double leftVelocityRotationsPerSecond = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTempCelcius = 0.0;
        public double rightPositionRotations = 0.0;
        public double rightVelocityRotationsPerSecond = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTempCelcius = 0.0;
    }

    public void setSysIdVoltage(Voltage volts);

}
