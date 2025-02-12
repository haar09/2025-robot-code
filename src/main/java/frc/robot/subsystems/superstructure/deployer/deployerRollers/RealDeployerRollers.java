package frc.robot.subsystems.superstructure.deployer.deployerRollers;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DeployerConstants;

public class RealDeployerRollers implements DeployerRollersIO {
    private final TalonFX rollerMotor = new TalonFX(DeployerConstants.kRollersMotorId);

    private final BaseStatusSignal rollerMotorVelocity, rollerMotorVoltage, rollerMotorTemp, rollerMotorSupplyCurrent;

    public RealDeployerRollers() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);

        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorVoltage = rollerMotor.getMotorVoltage();
        rollerMotorTemp = rollerMotor.getDeviceTemp();
        rollerMotorSupplyCurrent = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(250, rollerMotorVelocity, rollerMotorVoltage, rollerMotorTemp, rollerMotorSupplyCurrent);
        rollerMotor.optimizeBusUtilization();
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
    public void updateInputs(DeployerRollersIOInputs inputs){
        inputs.motorConnected = BaseStatusSignal.refreshAll(rollerMotorVelocity, rollerMotorVoltage, rollerMotorTemp, rollerMotorSupplyCurrent).isOK();

        inputs.velocityRotationsPerSecond = rollerMotorVelocity.getValueAsDouble();
        inputs.appliedVolts = rollerMotorVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = rollerMotorSupplyCurrent.getValueAsDouble();
        inputs.tempCelcius = rollerMotorTemp.getValueAsDouble();
    }
}