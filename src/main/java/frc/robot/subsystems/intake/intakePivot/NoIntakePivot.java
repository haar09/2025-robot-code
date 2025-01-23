package frc.robot.subsystems.intake.intakePivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class NoIntakePivot implements IntakePivotIO {

    public NoIntakePivot() {
    }

    @Override
    public void setDesiredAngle(Angle anglea){
    }

    @Override
    public Angle getAngle(){
        return null;
    }

    @Override
    public void stop(){
    }

    @Override
    public void setSysIdVoltage(Voltage volts){
    }

}