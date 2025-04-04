package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.stuypulse.stuylib.control.feedback.PIDController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoCommands.DriveKurzWithIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.superstructure.StateManager;

public class ObjectDetectionCmd extends Command {
    private final ObjectDetection objectDetection;
    private final CommandSwerveDrivetrain drivetrain;
    private final StateManager stateManager;
    private final XboxController controller;

    public ObjectDetectionCmd(ObjectDetection  objectDetection, CommandSwerveDrivetrain drivetrain, XboxController controller, StateManager stateManager) {
        this.objectDetection = objectDetection;
        this.drivetrain = drivetrain;
        this.stateManager = stateManager;
        this.controller = controller;
    }

    private final PIDController txPID = new PIDController(VisionConstants.kTXController_P.get(), 0, VisionConstants.kTXController_D.get());

    @Override
    public void initialize() {
        if (VisionConstants.kTXController_P.hasChanged(hashCode()) ||
        VisionConstants.kTXController_D.hasChanged(hashCode())
        ) {
            txPID.setPID(VisionConstants.kTXController_P.get(), 0, VisionConstants.kTXController_D.get());
        }

        txPID.reset();
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void execute() {
        double tx = objectDetection.getTX();
        double area = objectDetection.getArea();

                if (Math.abs(tx) < 5.5){ 
                    if (area > 6){
                        new DriveKurzWithIntake(drivetrain, stateManager).withTimeout(1.8).onlyWhile(() -> controller.getBButton()).schedule();
                        this.cancel();
                    }
                    drivetrain.setControl(drive.withVelocityX(-1.5)
                    .withVelocityY(0)
                    .withRotationalRate(txPID.update(0, tx))
                    );
                } else {
                    drivetrain.setControl(drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(txPID.update(0, tx))
                    );
                }
    }

    @Override
    public void end(boolean interrupted) {
        //controller.setRumble(RumbleType.kRightRumble, 0);
    }
}
