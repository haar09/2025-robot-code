package frc.robot.subsystems.intake.intakeRollers;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.NEO;

public class RealIntakeRollers implements IntakeRollersIO {
    private final NEO rollerMotor;

    public RealIntakeRollers(int motorId, boolean reversed) {
        rollerMotor = new NEO(motorId, reversed, IdleMode.kCoast);
    }

    @Override
    public void setOutputPercentage(double percentage) {
        rollerMotor.set(percentage);
    }

    @Override
    public void stop(){
        rollerMotor.set(0);
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs){
        inputs.motorConnected = rollerMotor.isConnected();

        inputs.velocityRotationsPerMinute = rollerMotor.getVelocity();
        inputs.appliedVolts = rollerMotor.getAppliedVoltage();
        inputs.supplyCurrentAmps = rollerMotor.getCurrent();
        inputs.tempCelcius = rollerMotor.getTemperature();
    }
}