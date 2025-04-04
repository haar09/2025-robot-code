package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.subsystems.superstructure.StateManager.State;

public class DriveKurzWithIntake extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;

    public DriveKurzWithIntake(CommandSwerveDrivetrain drivetrain, StateManager stateManager) {
        this.drivetrain = drivetrain;
        this.stateManager = stateManager;
    }

    @Override
    public void initialize() {
        timer = Timer.getFPGATimestamp();
        indiBile = false;
        debouncer = new Debouncer(0.25);
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(0).withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double timer;
    private boolean indiBile;
    private Debouncer debouncer;

    @Override
    public void execute() {
            stateManager.state = State.CORAL_INTAKE;
            if (debouncer.calculate(stateManager.intake.isAtDesiredAngle()) || indiBile){
            drivetrain.setControl(drive.withVelocityX(-2).withVelocityY(0).withRotationalRate(0));
            indiBile = true;
            } else if (Timer.getFPGATimestamp()-timer > 1) {
                stateManager.state = State.IDLE;
                this.cancel();
            }
    }

    @Override
    public void end(boolean interrupted){
        stateManager.state = State.IDLE;
    }
}
