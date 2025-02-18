package frc.robot.subsystems.superstructure.deployer.deployerOmnis;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DeployerConstants;

public class RealDeployerOmnis implements DeployerOmnisIO {
    private final TalonFX omniMotor = new TalonFX(DeployerConstants.kOmnisMotorId);

    private final BaseStatusSignal omniMotorVelocity, omniMotorVoltage, omniMotorTemp, omniMotorSupplyCurrent;

    public RealDeployerOmnis() {
        omniMotor.setNeutralMode(NeutralModeValue.Coast);

        omniMotorVelocity = omniMotor.getVelocity();
        omniMotorVoltage = omniMotor.getMotorVoltage();
        omniMotorTemp = omniMotor.getDeviceTemp();
        omniMotorSupplyCurrent = omniMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, omniMotorVelocity, omniMotorVoltage, omniMotorTemp, omniMotorSupplyCurrent);
        omniMotor.optimizeBusUtilization();
    }

    @Override
    public void setOutputPercentage(double percentage) {
        omniMotor.set(percentage);
    }

    @Override
    public void stop(){
        omniMotor.set(0);
    }

    @Override
    public void updateInputs(DeployerOmnisIOInputs inputs){
        inputs.motorConnected = BaseStatusSignal.refreshAll(omniMotorVelocity, omniMotorVoltage, omniMotorTemp, omniMotorSupplyCurrent).isOK();

        inputs.velocityRotationsPerSecond = omniMotorVelocity.getValueAsDouble();
        inputs.appliedVolts = omniMotorVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = omniMotorSupplyCurrent.getValueAsDouble();
        inputs.tempCelcius = omniMotorTemp.getValueAsDouble();
    }
}