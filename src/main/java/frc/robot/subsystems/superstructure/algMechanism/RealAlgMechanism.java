package frc.robot.subsystems.superstructure.algMechanism;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.NEO;

public class RealAlgMechanism implements AlgMechanismIO{
    private final NEO rackMotor/*, rollerMotor*/;
    private final SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(10);

    public RealAlgMechanism(int rackMotorId/*, int rollerMotorId*/){
        this.rackMotor = new NEO(rackMotorId, false, IdleMode.kBrake);
        //this.rollerMotor = new NEO(rollerMotorId, false, IdleMode.kCoast);
        rackMotor.config.apply(config);
    }

    @Override
    public void setDesiredExtension(double extension){
        rackMotor.set(extension);
    }

    /*@Override
    public void setOutputPercentage(double percentage){
        rollerMotor.set(percentage);
    }*/

    @Override
    public void updateInputs(AlgMechanismIOInputs inputs){
        inputs.rackMotorConnected = rackMotor.isConnected();
        //inputs.rollerMotorConnected = rollerMotor.isConnected();

        inputs.rackPositionRotations = rackMotor.getPosition();
        inputs.rackVelocityRPM = rackMotor.getVelocity();
        inputs.rackAppliedVolts = rackMotor.getAppliedVoltage();
        inputs.rackCurrentAmps = rackMotor.getCurrent();
        inputs.rackTemperatureCelcius = rackMotor.getTemperature();
        /*inputs.rollerPositionRotations = rollerMotor.getPosition();
        inputs.rollerVelocityRPM = rollerMotor.getVelocity();
        inputs.rollerAppliedVolts = rollerMotor.getAppliedVoltage();
        inputs.rollerCurrentAmps = rollerMotor.getCurrent();
        inputs.rollerTemperatureCelcius = rollerMotor.getTemperature();*/
    }
}