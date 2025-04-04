package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.algMechanism.AlgMechanism;

public class AlgRetract extends Command{
    private final AlgMechanism algMechanism;

    public AlgRetract(AlgMechanism algMechanism){
        this.algMechanism = algMechanism;
        addRequirements(algMechanism);
    }

    @Override
    public void execute(){
        algMechanism.setDesiredExtension(-0.35);
    }

    @Override
    public void end(boolean interrupted){
        algMechanism.setDesiredExtension(0);
    }
}
