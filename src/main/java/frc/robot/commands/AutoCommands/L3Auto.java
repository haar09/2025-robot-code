package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.StateManager;
import frc.robot.subsystems.superstructure.StateManager.State;

public class L3Auto extends Command{
    private final StateManager stateManager;
    private final CommandSwerveDrivetrain drivetrain;

    public L3Auto(StateManager stateManager, CommandSwerveDrivetrain drivetrain) {
        this.stateManager = stateManager;
        this.drivetrain = drivetrain;
    }

    private Pose2d goalPosition;
    private boolean leftInstead = false;

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;

                    goalPosition = Reef.scoringPositions2d.get(FieldConstants.findClosestReefside(currentPose)*2).get(ReefLevel.L23);


                double angleDifference = goalPosition.getRotation().plus(Rotation2d.kCCW_90deg).minus(currentPose.getRotation()).getDegrees();

            if ((GlobalVariables.getInstance().alliance == Alliance.Blue && angleDifference < 0) ||
            (GlobalVariables.getInstance().alliance != Alliance.Blue && angleDifference > 0)) {
                leftInstead = true;
                goalPosition = goalPosition.transformBy(new Transform2d(-0.36, 0, new Rotation2d(Units.degreesToRadians(180))));
            } else {
                leftInstead = false;
            }

    }

    @Override
    public void execute() {
        if (leftInstead){
            stateManager.state = State.L3_LEFT;
        } else {
        stateManager.state = State.L3_RIGHT;
        }
    }

    @Override
    public void end(boolean interrupted) {
        stateManager.state = State.IDLE;
    }

    @Override
    public boolean isFinished() {
        return !stateManager.deployer.deployerBeamBreak.left_value && !stateManager.deployer.deployerBeamBreak.right_value;
    }
}
