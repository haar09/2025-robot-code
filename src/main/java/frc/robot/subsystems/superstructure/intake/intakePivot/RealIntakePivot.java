package frc.robot.subsystems.superstructure.intake.intakePivot;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;

public class RealIntakePivot implements IntakePivotIO {
    
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
    private final PositionVoltage positionVoltage;
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
        
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            pivotMotorPosition,
            pivotMotorVelocity,
            pivotMotorVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            pivotMotorTemp,
            pivotMotorSupplyCurrent
        );

        pivotMotor.optimizeBusUtilization();
        resetEncoders();

        positionVoltage = new PositionVoltage(getAbsolutePosition());
    }

    @Override
    public void setDesiredAngle(Angle angle){
        pivotMotor.setControl(positionVoltage.withPosition(angle));
    }

    @Override
    public Angle getAngle(){
        return pivotMotor.getPosition().getValue();
    }

    public Angle getAbsolutePosition(){
        return Rotations.of(throughBoreEncoder.get());
    }

    public void resetEncoders(){
       pivotMotor.setPosition(getAbsolutePosition(), 1);
    }

    
    @Override
    public void stop(){
        pivotMotor.set(0);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs){
        inputs.motorConnected = BaseStatusSignal.refreshAll(pivotMotorVelocity, pivotMotorVoltage, pivotMotorTemp, pivotMotorSupplyCurrent).isOK();
        inputs.absoluteEncoderConnected = throughBoreEncoder.isConnected();

        inputs.positionRads = getAngle().in(Radians);
        inputs.absoluteEncoderPositionRads = getAbsolutePosition().in(Radians);
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