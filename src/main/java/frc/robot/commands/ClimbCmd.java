package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbCmd extends Command{
    private final Climb climb;
    private final Supplier<Boolean> take, out;

    public ClimbCmd(Supplier<Boolean> take, Supplier<Boolean> out, Climb climb){
        this.take = take;
        this.out = out;
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        // get joystick
        boolean takeSpeed = take.get();
        boolean outSpeed = out.get();

        if (takeSpeed) {
            climb.setOutputPercentage(1);
        } else if (outSpeed) {
                climb.setOutputPercentage(-0.6);
        } else {
            climb.setOutputPercentage(0);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}