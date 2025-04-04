package frc.robot.subsystems.superstructure.intake.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.MotionProfileConstants;

public class RealIntakePivot implements IntakePivotIO {
    
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
    private final BaseStatusSignal
    pivotMotorPosition, pivotMotorVelocity, pivotMotorVoltage, pivotMotorTemp, pivotMotorSupplyCurrent;

    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(IntakeConstants.kAbsoluteEncoderId);

    private Controller controller;

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
            pivotMotorVoltage,
            pivotMotorTemp,
            pivotMotorSupplyCurrent
        );

        pivotMotor.optimizeBusUtilization();
        throughBoreEncoder.setInverted(false);
        throughBoreEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        throughBoreEncoder.setAssumedFrequency(975.6);
        
        controller = new PIDController(MotionProfileConstants.kP, MotionProfileConstants.kI, MotionProfileConstants.kD);
    }

    @Override
    public void setDesiredAngle(Angle angle){
        pivotMotor.setVoltage(controller.update(angle.in(Degrees), getAngle().in(Degrees)));
    }

    @Override
    public Angle getAngle(){
        return getAbsolutePosition();
    }

    @Override
    public Angle getAbsolutePosition(){
        return Rotations.of(throughBoreEncoder.get()+IntakeConstants.kAbsoluteEncoderOffset);
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

        inputs.positionRots = pivotMotorPosition.getValueAsDouble();
        inputs.absoluteEncoderPositionRots = throughBoreEncoder.get();
        inputs.quadratureEncoderPositionRots = getAngle().in(Rotations);
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