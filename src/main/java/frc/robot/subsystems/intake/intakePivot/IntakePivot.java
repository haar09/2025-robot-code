package frc.robot.subsystems.intake.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

public class IntakePivot extends SubsystemBase{
    private final IntakePivotIO pivot;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
    private Pose3d intakePose = new Pose3d(0, 0, 0, new Rotation3d(0,0,0));
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
        new SysIdRoutine.Config(Volts.of(0.5).per(Second),        // Use default ramp rate (1 V/s)
        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
        null,        // Use default timeout (10 s)
        // Log state with SignalLogger class
        state -> SignalLogger.writeString("SysIdArm_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts-> pivot.setSysIdVoltage(volts),
            null,
            this
        )
        );
    }

    @Override
    public void periodic() {
        pivot.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
        disconnectedAlert.set(!inputs.motorConnected);

        intakePose = new Pose3d(
            0,
            0,
            0,
            new Rotation3d(0, -pivot.getAngle().in(Radians), 0));
            
        Logger.recordOutput("IntakePivot/Angle", intakePose);
        
        positionVisualizer.setState(getAngle());
        Logger.recordOutput("IntakePivot/Position", positionVisualizer.mech);
        Logger.recordOutput("IntakePivot/SetPoint", setPointVisualizer.mech);

        
        setPointVisualizer.setState(0);

        SmartDashboard.putNumber("IntakePivot Angle", pivot.getAngle().in(Degrees));
    }

    public void setDesiredAngle(Angle angle) {
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), IntakeConstants.kMinIntakeAngleDegrees, IntakeConstants.kMaxIntakeAngleDegrees));
        pivot.setDesiredAngle(angle);
        setPointVisualizer.setState(angle.in(Radians));
    }

    public void stop() {
        pivot.stop();
    }

    public double getAngle() {
        return pivot.getAngle().in(Radians);
    }

    private final SysIdRoutine sysIdRoutine;

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}