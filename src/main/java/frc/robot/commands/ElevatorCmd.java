package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class ElevatorCmd extends Command {
    private final Elevator elevator;
    private final Supplier<String> upDownInput;

    public ElevatorCmd(Elevator elevator, Supplier<String> upDownInput) {
        this.elevator = elevator;
        this.upDownInput = upDownInput;
    }

    @Override
    public void initialize() {
        elevator.setVoltage(0);
    }
 
    @Override
    public void execute() {
        var upDown = upDownInput.get();
        if (upDown.contentEquals("U")) {
            elevator.setVoltage(2);
        } else if (upDown.contentEquals("D")) {
            elevator.setVoltage(-2);
        } else {
            elevator.setVoltage(0);
        }
    }

}
