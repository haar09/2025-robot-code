package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class RealElevator implements ElevatorIO {
    private final TalonFX leftMotorLeader = new TalonFX(ElevatorConstants.kLeftMotorId);
    private final TalonFX rightMotorFollower = new TalonFX(ElevatorConstants.kRightMotorId);

    private final BaseStatusSignal leftMotorPosition, leftMotorVelocity, leftMotorVoltage, leftMotorTemp, leftMotorSupplyCurrent,
                                    rightMotorPosition, rightMotorVelocity, rightMotorVoltage, rightMotorTemp, rightMotorSupplyCurrent;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public RealElevator() {
        leftMotorLeader.getConfigurator().apply(ElevatorConstants.elevatorMotorConfig);
        rightMotorFollower.getConfigurator().apply(ElevatorConstants.elevatorMotorConfig);
    
        leftMotorPosition = leftMotorLeader.getPosition();
        leftMotorVelocity = leftMotorLeader.getVelocity();
        leftMotorVoltage = leftMotorLeader.getMotorVoltage();
        leftMotorTemp = leftMotorLeader.getDeviceTemp();
        leftMotorSupplyCurrent = leftMotorLeader.getSupplyCurrent();
        rightMotorPosition = rightMotorFollower.getPosition();
        rightMotorVelocity = rightMotorFollower.getVelocity();
        rightMotorVoltage = rightMotorFollower.getMotorVoltage();
        rightMotorTemp = rightMotorFollower.getDeviceTemp();
        rightMotorSupplyCurrent = rightMotorFollower.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, 
            leftMotorPosition,
            leftMotorVelocity,
            leftMotorVoltage,
            rightMotorPosition,
            rightMotorVelocity,
            rightMotorVoltage,
            leftMotorTemp,
            leftMotorSupplyCurrent,
            rightMotorTemp,
            rightMotorSupplyCurrent
        );

        leftMotorLeader.optimizeBusUtilization();
        rightMotorFollower.optimizeBusUtilization();

        leftMotorLeader.setPosition(0);
        rightMotorFollower.setPosition(0);
    }

    @Override
    public void setPosition(double rotations) {
        leftMotorLeader.setControl(motionMagicVoltage.withPosition(rotations));
        rightMotorFollower.setControl(new Follower(leftMotorLeader.getDeviceID(), true));
    }

    @Override
    public void stop() {
        leftMotorLeader.setControl(new NeutralOut());
        rightMotorFollower.setControl(new NeutralOut());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(leftMotorPosition, leftMotorVelocity, leftMotorVoltage, leftMotorTemp, leftMotorSupplyCurrent).isOK();
        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(rightMotorPosition, rightMotorVelocity, rightMotorVoltage, rightMotorTemp, rightMotorSupplyCurrent).isOK();

        inputs.leftPositionRotations = leftMotorPosition.getValueAsDouble();
        inputs.leftVelocityRotationsPerSecond = leftMotorVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftMotorVoltage.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftMotorSupplyCurrent.getValueAsDouble();
        inputs.leftTempCelcius = leftMotorTemp.getValueAsDouble();

        inputs.rightPositionRotations = rightMotorPosition.getValueAsDouble();
        inputs.rightVelocityRotationsPerSecond = rightMotorVelocity.getValueAsDouble();
        inputs.rightAppliedVolts = rightMotorVoltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightMotorSupplyCurrent.getValueAsDouble();
        inputs.rightTempCelcius = rightMotorTemp.getValueAsDouble();
    }

    private final VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public void setSysIdVoltage(Voltage volts){
        leftMotorLeader.setControl(voltageOut.withOutput(volts));
        rightMotorFollower.setControl(new Follower(leftMotorLeader.getDeviceID(), true));
    }
}
