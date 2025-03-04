package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.algMechanism.AlgMechanism;

public class AlgMechanismCmd extends Command{
    private final AlgMechanism algMechanism;
    private final Supplier<Boolean> extend, retract;

    public AlgMechanismCmd(AlgMechanism algMechanism, Supplier<Boolean> extend, Supplier<Boolean> retract){
        this.algMechanism = algMechanism;
        this.extend = extend;
        this.retract = retract;
        addRequirements(algMechanism);
    }

    @Override
    public void execute(){
        boolean extendButton = extend.get();
        boolean retractButton = retract.get();

        if (extendButton) {
            algMechanism.setDesiredExtension(0.35);
            algMechanism.setOutputPercentage(1);
        } else if (retractButton) {
            algMechanism.setDesiredExtension(-0.35);
        } else {
            algMechanism.setDesiredExtension(0);
            algMechanism.setOutputPercentage(0);
        }
    }
}
