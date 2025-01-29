package frc.robot.subsystems.intake.intakePivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class NoIntakePivot implements IntakePivotIO {
    private Angle m_angle = Degrees.of(0);
    private SlewRateLimiter limiter = new SlewRateLimiter(90);

    public NoIntakePivot() {
        limiter.reset(0);
    }

    @Override
    public void setDesiredAngle(Angle angle){
        m_angle = Degrees.of(limiter.calculate(angle.in(Degrees)));
    }

    @Override
    public Angle getAngle(){
        return m_angle;
    }

    @Override
    public void stop(){
    }

    @Override
    public void setSysIdVoltage(Voltage volts){
    }

}