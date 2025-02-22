package frc.robot.subsystems.superstructure.intake.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;

public class RealIntakePivot implements IntakePivotIO {
    
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
    private final MotionMagicVoltage positionVoltage;
    private final BaseStatusSignal
    pivotMotorPosition, pivotMotorVelocity, pivotMotorVoltage, pivotMotorTemp, pivotMotorSupplyCurrent;

    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(IntakeConstants.kAbsoluteEncoderId);

    public RealIntakePivot() {
        pivotMotor.getConfigurator().apply(IntakeConstants.pivotMotorConfig);
        
        pivotMotorPosition = pivotMotor.getPosition();
        pivotMotorVelocity = pivotMotor.getVelocity();
        pivotMotorVoltage = pivotMotor.getMotorVoltage();
        pivotMotorTemp = pivotMotor.getDeviceTemp();
        pivotMotorSupplyCurrent = pivotMotor.getSupplyCurrent();
        
        BaseStatusSignal.setUpdateFrequencyForAll(50,
            pivotMotorPosition,
            pivotMotorVelocity,
            pivotMotorVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            pivotMotorTemp,
            pivotMotorSupplyCurrent
        );

        pivotMotor.optimizeBusUtilization();
        throughBoreEncoder.setInverted(true);
        throughBoreEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        throughBoreEncoder.setAssumedFrequency(975.6);
        positionVoltage = new MotionMagicVoltage(IntakeConstants.idleAngle);
        resetEncoders();

        new Thread (() -> {
            while (true) {
                if (throughBoreEncoder.isConnected()) {
                    try {
                        Thread.sleep(500);
                        resetEncoders();
                        break;
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).run();
    }

    @Override
    public void setDesiredAngle(Angle angle){
        pivotMotor.setControl(positionVoltage.withPosition(angle));
    }

    @Override
    public Angle getAngle(){
        return pivotMotor.getPosition().getValue()/*.div(IntakeConstants.kTotalRatio)*/;
    }

    @Override
    public Angle getAbsolutePosition(){
        return Rotations.of(throughBoreEncoder.get()/IntakeConstants.kEncoderToPivot).plus(Degrees.of(IntakeConstants.kAbsoluteEncoderOffset));
    }

    @Override
    public void resetEncoders(){
       pivotMotor.setPosition(getAbsolutePosition(), 1).toString();
       //pivotMotor.setPosition(IntakeConstants.idleAngle.in(Rotations));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralModeValue){
        pivotMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void stop(){
        pivotMotor.set(0);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs){
        inputs.motorConnected = BaseStatusSignal.refreshAll(pivotMotorPosition, pivotMotorVelocity, pivotMotorVoltage, pivotMotorTemp, pivotMotorSupplyCurrent).isOK();
        inputs.absoluteEncoderConnected = throughBoreEncoder.isConnected();

        inputs.positionRads = getAngle().in(Radians);
        inputs.absoluteEncoderPositionRots = throughBoreEncoder.get();
        inputs.velocityRotsPerSec = pivotMotorVelocity.getValueAsDouble();
        inputs.appliedVolts = pivotMotorVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = pivotMotorSupplyCurrent.getValueAsDouble();
        inputs.tempCelcius = pivotMotorTemp.getValueAsDouble();
    }

    private final VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public void setSysIdVoltage(Voltage volts){
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }

}