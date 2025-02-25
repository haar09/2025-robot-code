package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NetworkTablesAgent;

public class AutoBranchandShootL23 extends Command{
    private NetworkTablesAgent networkTablesAgent;
    private CommandSwerveDrivetrain drivetrain;

    private String reefBranch;
    private double reefLevel;

    private Pose2d goalPosition;
    private boolean leftInstead = false;

    public AutoBranchandShootL23(NetworkTablesAgent networkTablesAgent, CommandSwerveDrivetrain drivetrain) {
        this.networkTablesAgent = networkTablesAgent;
    }

    @Override
    public void initialize() {
        reefBranch = networkTablesAgent.buttonValue.get();
        reefLevel = networkTablesAgent.triggerValue.get();

            if(reefBranch.contentEquals("A")){
                goalPosition = Reef.scoringPositions2d.get(0).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("B")){
                goalPosition = Reef.scoringPositions2d.get(1).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("C")){
                goalPosition = Reef.scoringPositions2d.get(2).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("D")){
                goalPosition = Reef.scoringPositions2d.get(3).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("E")){
                goalPosition = Reef.scoringPositions2d.get(4).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("F")){
                goalPosition = Reef.scoringPositions2d.get(5).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("G")){
                goalPosition = Reef.scoringPositions2d.get(6).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("H")){
                goalPosition = Reef.scoringPositions2d.get(7).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("I")){
                goalPosition = Reef.scoringPositions2d.get(8).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("J")){
                goalPosition = Reef.scoringPositions2d.get(9).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("K")){
                goalPosition = Reef.scoringPositions2d.get(10).get(ReefLevel.L23);
            } else if (reefBranch.contentEquals("L")){
                goalPosition = Reef.scoringPositions2d.get(11).get(ReefLevel.L23);
            }

            List<Pose2d> flippedCenterFaces = Arrays.stream(FieldConstants.Reef.centerFaces)
            .map(AllianceFlipUtil::apply)
            .collect(Collectors.toList());

            if ((drivetrain.getState().Pose.getRotation().getDegrees() 
            - drivetrain.getState().Pose.nearest(flippedCenterFaces).getRotation().getDegrees() 
            + 360) % 360 > 180){
            } else {
                leftInstead = true;
                goalPosition.transformBy(new Transform2d(0.115, 0, new Rotation2d(180)));
            }
        }

    @Override
    public void execute() {

    }
}