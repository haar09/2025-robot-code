package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private final TalonFX leftMotorLeader = new TalonFX(ElevatorConstants.kLeftMotorId);
    private final TalonFX rightMotorFollower = new TalonFX(ElevatorConstants.kRightMotorId);

    private Distance lastDesiredPosition = Meters.of(0);

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Elevator() {
        leftMotorLeader.getConfigurator().apply(ElevatorConstants.elevatorMotorConfig);
        rightMotorFollower.getConfigurator().apply(ElevatorConstants.elevatorMotorConfig);
    }

    public Distance getElevatorPosition() {
        return Meters.of(leftMotorLeader.getPosition().getValueAsDouble());
    }

    public boolean isAtSetpoint() {
        return getElevatorPosition().isNear(lastDesiredPosition, ElevatorConstants.kDistanceTolerance);
    }

    public void setPosition(Distance height) {
        leftMotorLeader.setControl(motionMagicVoltage.withPosition(height.in(Meters)));
        rightMotorFollower.setControl(new Follower(leftMotorLeader.getDeviceID(), true));
        lastDesiredPosition = height;
    }

    public void stop() {
        leftMotorLeader.setControl(new NeutralOut());
        rightMotorFollower.setControl(new NeutralOut());
    }

    
}
