package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ModifiedSignalLogger;
import frc.robot.Robot;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private final Alert disconnectedAlertLeft, disconnectedAlertRight;

    @AutoLogOutput private Distance lastDesiredPosition = Centimeters.of(0);

    private ElevatorVisualizer elevatorPositionVisualizer = new ElevatorVisualizer();

    public static Elevator create() {
        return new Elevator(Robot.isReal() ? new RealElevator() : new NoElevator());
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.disconnectedAlertLeft = new Alert("Elevator Left is disconnected.", AlertType.kWarning);
        this.disconnectedAlertRight = new Alert("Elevator Right is disconnected.", AlertType.kWarning);
        sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.5).div(Seconds.of(1)),        // Use default ramp rate (1 V/s)
        Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
        Seconds.of(3),        // Use default timeout (10 s)
        // Log state with SignalLogger class
        ModifiedSignalLogger.logState()
        ),
        new SysIdRoutine.Mechanism(
            volts-> io.setSysIdVoltage(volts),
            null,
            this
        )
        );
        SmartDashboard.putNumber("asansoryukseklik", 18);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        disconnectedAlertLeft.set(!inputs.leftMotorConnected);
        disconnectedAlertRight.set(!inputs.rightMotorConnected);
        Logger.recordOutput("Elevator/HeightCM", getElevatorPosition().in(Centimeters));
        Logger.recordOutput("Elevator/Position", elevatorPositionVisualizer.mechanism);    
    }

    public Distance getElevatorPosition() {
        return Centimeters.of(inputs.leftPositionRotations * ElevatorConstants.kElevatorRotToCm);
    }

    public boolean isAtSetpoint() {
        return getElevatorPosition().isNear(lastDesiredPosition, ElevatorConstants.kDistanceTolerance);
    }

    public void setPosition(Distance height) {
        io.setPosition(height.in(Centimeters)/ElevatorConstants.kElevatorRotToCm);
        elevatorPositionVisualizer.setState(height.in(Meters));
        lastDesiredPosition = height.copy();
    }

    public void stop() {
        io.stop();
    }
    
    private final SysIdRoutine sysIdRoutine;

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setVoltage(double voltage) {
        io.setSysIdVoltage(Volts.of(voltage));
    }
}
