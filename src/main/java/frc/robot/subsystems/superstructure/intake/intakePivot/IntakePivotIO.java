package frc.robot.subsystems.superstructure.intake.intakePivot;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    public void setDesiredAngle(Angle angle);
    public void stop();
    public Angle getAngle();
    public void setNeutralMode(NeutralModeValue neutralModeValue);
    public Angle getAbsolutePosition();

    default void updateInputs(IntakePivotIOInputs inputs) {}

    @AutoLog
    class IntakePivotIOInputs {
        public boolean motorConnected = true;

        public double positionRots = 0.0;
        public double absoluteEncoderPositionRots = 0.0;
        public double quadratureEncoderPositionRots = 0.0;
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
        public boolean absoluteEncoderConnected = true;
    }

    public void setSysIdVoltage(Voltage volts);
}