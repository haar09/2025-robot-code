package frc.robot.subsystems.superstructure.intake.intakePivot;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakePivotVisualizer implements Sendable, AutoCloseable {
  public final LoggedMechanism2d mech;
  private final LoggedMechanismLigament2d arm;

  public IntakePivotVisualizer(Color8Bit color) {
    mech = new LoggedMechanism2d(2, 2, new Color8Bit(169,169,169));
    LoggedMechanismRoot2d chassis =
        mech.getRoot("Chassis", 1-0.125, 0.33);
    arm = chassis.append(new LoggedMechanismLigament2d("Intake", -0.3, 0, 4, color));
    arm.append(new LoggedMechanismLigament2d("IntakeEnd", 0.2, -135, 4, color));
  }

  public void setState(double angle) {
    arm.setAngle(-Units.radiansToDegrees(angle));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }

  @Override
  public void close() throws Exception {
    mech.close();
  }
}