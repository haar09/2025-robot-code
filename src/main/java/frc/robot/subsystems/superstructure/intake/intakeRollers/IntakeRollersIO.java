package frc.robot.subsystems.superstructure.intake.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(IntakeRollersIOInputs inputs) {}

    @AutoLog
    class IntakeRollersIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerMinute = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
}