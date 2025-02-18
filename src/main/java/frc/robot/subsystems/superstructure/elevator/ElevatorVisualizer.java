package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer implements Sendable, AutoCloseable {
        public final LoggedMechanism2d mechanism = new LoggedMechanism2d(0.7, 1.8);

        private final LoggedMechanismRoot2d elevatorLeftBase  = mechanism.getRoot("Left Elevator Root", 0.165, 0);
        private final LoggedMechanismRoot2d elevatorRightBase = mechanism.getRoot("Right Elevator Root", 0.535, 0);

        private LoggedMechanismRoot2d _leftStageOneRoot   = mechanism.getRoot("Left Stage One Root", 0.19, 0.025);
        private LoggedMechanismRoot2d _rightStageOneRoot  = mechanism.getRoot("Right Stage One Root", 0.51, 0.025);
        private LoggedMechanismRoot2d _carriageBottomRoot = mechanism.getRoot("Carriage Bottom Root", 0.23, 0.05);

        private final LoggedMechanismLigament2d carriageBottom = new LoggedMechanismLigament2d("Carriage Bottom", 0.2419, 0, 2.5, new Color8Bit(Color.kOrange));
        private final LoggedMechanismLigament2d carriageRight = new LoggedMechanismLigament2d("Carriage Right", 0.14, 90, 2.5, new Color8Bit(Color.kOrange));
        private final LoggedMechanismLigament2d carriageTop = new LoggedMechanismLigament2d("Carriage Top", 0.2419, 90, 2.5, new Color8Bit(Color.kOrange));
        private final LoggedMechanismLigament2d carriageLeft   = new LoggedMechanismLigament2d("Carriage Left", 0.14, 90, 2.5, new Color8Bit(Color.kOrange));

        public ElevatorVisualizer() {
            elevatorLeftBase.append(new LoggedMechanismLigament2d("Left Base", 0.75, 90, 2.5, new Color8Bit(Color.kOrange)));
            elevatorRightBase.append(new LoggedMechanismLigament2d("Right Static Stage One", 0.75, 90, 2.5, new Color8Bit(Color.kOrange)));
            _leftStageOneRoot.append(new LoggedMechanismLigament2d("Left Stage One Root", 0.715, 90, 2.5, new Color8Bit(Color.kOrange)));
            _rightStageOneRoot.append(new LoggedMechanismLigament2d("Right Stage One Root", 0.715, 90, 2.5, new Color8Bit(Color.kOrange)));
            _carriageBottomRoot.append(carriageBottom);
            carriageBottom.append(carriageRight);
            carriageRight.append(carriageTop);
            carriageTop.append(carriageLeft);
        }
        
        public void setState(double elevatorPosition) {
            _leftStageOneRoot.setPosition(0.19, elevatorPosition + 0.025);
            _rightStageOneRoot.setPosition(0.51, elevatorPosition + 0.025);
            _carriageBottomRoot.setPosition(0.23, elevatorPosition*2 + 0.05);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            mechanism.initSendable(builder);
        }
      
        @Override
        public void close() throws Exception {
            mechanism.close();
        }
}
