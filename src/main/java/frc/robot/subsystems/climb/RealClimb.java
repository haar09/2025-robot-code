package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimbConstants;

public class RealClimb implements ClimbIO{
        private final TalonFX climbMotor = new TalonFX(0);

        private final BaseStatusSignal
         climbMotorVelocity,
         climbMotorVoltage,
         climbMotorTemp,
         climbMotorSupplyCurrent;

        public RealClimb() {
            climbMotor.setNeutralMode(NeutralModeValue.Brake);

            climbMotor.getConfigurator().apply(ClimbConstants.climbMotorConfig);

            climbMotorVelocity = climbMotor.getVelocity();
            climbMotorVoltage = climbMotor.getMotorVoltage();
            climbMotorTemp = climbMotor.getDeviceTemp();
            climbMotorSupplyCurrent = climbMotor.getSupplyCurrent();

            BaseStatusSignal.setUpdateFrequencyForAll(100,
            climbMotorVelocity,
            climbMotorVoltage,
            climbMotorTemp,
            climbMotorSupplyCurrent);

            climbMotor.optimizeBusUtilization();
        }

        @Override
        public void updateInputs(ClimbIOInputs inputs){
            inputs.motorConnected = BaseStatusSignal.refreshAll(climbMotorVelocity,
            climbMotorVoltage,
            climbMotorTemp,
            climbMotorSupplyCurrent).isOK();


            inputs.climbMotorVelocityRps = climbMotorVelocity.getValueAsDouble();

            inputs.climbMotorAppliedVolts = climbMotorVoltage.getValueAsDouble();

            inputs.climbMotorSupplyCurrentAmps = climbMotorSupplyCurrent.getValueAsDouble();

            inputs.climbMotorTempCelsius = climbMotorTemp.getValueAsDouble();
        }

        @Override
        public void setOutputPercentage(double percentage) {
            climbMotor.set(percentage);
        }

        @Override
        public void stop() {
            climbMotor.set(0);
        }


}