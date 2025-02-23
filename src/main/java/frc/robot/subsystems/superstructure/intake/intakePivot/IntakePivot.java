package frc.robot.subsystems.superstructure.intake.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ModifiedSignalLogger;

public class IntakePivot extends SubsystemBase{
    private final IntakePivotIO pivot;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
    //private Pose3d intakePose = new Pose3d(0, 0, 0, new Rotation3d(0,0,0));
    private final Alert disconnectedAlert;

    private final IntakePivotVisualizer positionVisualizer = new IntakePivotVisualizer(new Color8Bit(255, 0, 0));
    private final IntakePivotVisualizer setPointVisualizer = new IntakePivotVisualizer(new Color8Bit(0, 0, 255));

    public static IntakePivot create() {
        return new IntakePivot(Robot.isReal() ? new RealIntakePivot() : new NoIntakePivot());
    }

    public IntakePivot(IntakePivotIO pivot) {
        this.pivot = pivot;
        this.disconnectedAlert = new Alert("IntakePivot is disconnected.", AlertType.kWarning);
        sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null,        // Use default ramp rate (1 V/s)
        Volts.of(1.7), // Reduce dynamic step voltage to 4 V to prevent brownout
        Seconds.of(2),        // Use default timeout (10 s)
        // Log state with SignalLogger class
        ModifiedSignalLogger.logState()
        ),
        new SysIdRoutine.Mechanism(
            volts-> pivot.setSysIdVoltage(volts),
            null,
            this
        )
        );
        setPointVisualizer.setState(0);
        lastDesiredAngle = IntakeConstants.idleAngle;
    }

    @Override
    public void periodic() {
        pivot.updateInputs(inputs);
        Logger.processInputs("Intake/Pivot", inputs);
        disconnectedAlert.set(!inputs.motorConnected);

        /*intakePose = new Pose3d(
            0,
            0,
            0,
            new Rotation3d(0, -pivot.getAngle().in(Radians), 0));
            
        Logger.recordOutput("Intake/Pivot/Angle", intakePose);*/
        
        positionVisualizer.setState(getAngle().in(Radians));
        Logger.recordOutput("Intake/Pivot/Position", positionVisualizer.mech);
        Logger.recordOutput("Intake/Pivot/SetPoint", setPointVisualizer.mech);
        SmartDashboard.putNumber("Intake Pivot Angle", getAngle().in(Degrees));
        Logger.recordOutput("Intake/Pivot/Last Desired Angle", lastDesiredAngle.in(Degrees));
    }

    private Angle lastDesiredAngle;

    public void setDesiredAngle(Angle angle) {
        pivot.setDesiredAngle(angle);
        setPointVisualizer.setState(angle.in(Radians));
        lastDesiredAngle = angle.copy();
    }

    public void setSlowAngle(Angle angle) {
        pivot.setSlowAngle(angle);
        setPointVisualizer.setState(angle.in(Radians));
        lastDesiredAngle = angle.copy();
    }

    @AutoLogOutput
    public boolean isAtDesiredAngle() {
        return getAngle().isNear(lastDesiredAngle, IntakeConstants.kAngleTolerance);
    }

    public void setCoast() {
        pivot.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBrake() {
        pivot.setNeutralMode(NeutralModeValue.Brake);
    }

    public void stop() {
        pivot.stop();
    }

    public Angle getAngle() {
        return pivot.getAngle();
    }

    public Angle getAbsoluteAngle(){
        return pivot.getAbsolutePosition();
    }

    public void resetEncoders(){
        //Logger.recordOutput("absolutencoder", pivot.getAbsolutePosition().in(Degrees));
        pivot.resetEncoders();
    }

    public double getVelocity() {
        return inputs.velocityRotsPerSec;
    }

    private final SysIdRoutine sysIdRoutine;

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}