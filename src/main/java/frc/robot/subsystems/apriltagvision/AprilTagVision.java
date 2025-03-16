package frc.robot.subsystems.apriltagvision;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AprilTagVision extends SubsystemBase{
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public AprilTagVision(CommandSwerveDrivetrain drivetrain, AprilTagVisionIO... io){
        this.drivetrain = drivetrain;
        this.io = io;

        this.inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new AprilTagVisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
        disconnectedAlerts[i] =
            new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    List<Integer> tagsToReject = Arrays.asList();

    @Override
    public void periodic(){
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/OV9281_" + Integer.toString(i), inputs[i]);
        }

        SmartDashboard.putBoolean("Camera Target", false);

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();
            
            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.kTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                tagPoses.add(tagPose.get());
                }
            }

            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));

            for (var observation : inputs[cameraIndex].poseObservations) {
                SmartDashboard.putBoolean("Camera Target", true);
                if (observation.estPose() == null) {
                    continue;
                }

                DriverStation.getAlliance().ifPresent((allianceColor) -> {
                    if (allianceColor == Alliance.Red) {
                        tagsToReject = Arrays.asList(17,18,19,20,21,22,4,5);
                        } else {
                        tagsToReject = Arrays.asList(6,7,8,9,10,11,14,15);
                        }
                    });

                boolean hasRejectedTag = false;
                if (cameraIndex>1) {
                for (var target : observation.estPose().targetsUsed) {
                    if (tagsToReject.contains(target.fiducialId)) {
                        hasRejectedTag = true;
                        break;
                    }
                }}

                boolean rejectPose =
                    observation.estPose().targetsUsed.size() == 0 // Must have at least one tag

                    || (observation.tagCount() == 1
                        && observation.ambiguity() > VisionConstants.kMaxAmbiguity) // Cannot be high ambiguity

                    || Math.abs(observation.estPose().estimatedPose.getZ())
                        > VisionConstants.kMaxZError // Must have realistic Z coordinate
    
                    // Must be within the field boundaries
                    || observation.estPose().estimatedPose.getX() < 0.0
                    || observation.estPose().estimatedPose.getX() > VisionConstants.kTagLayout.getFieldLength()
                    || observation.estPose().estimatedPose.getY() < 0.0
                    || observation.estPose().estimatedPose.getY() > VisionConstants.kTagLayout.getFieldWidth()
                    || hasRejectedTag;

                if (rejectPose) {
                robotPosesRejected.add(observation.estPose().estimatedPose);
                } else {
                robotPosesAccepted.add(observation.estPose().estimatedPose);
                }

                if (rejectPose) {
                    continue;
                }

                double linearStdDev, stDevFactor, angularStdDev;

                if (DriverStation.isEnabled()) {
                stDevFactor = Math.pow(observation.averageTagDistance(), 2) / observation.tagCount();
                linearStdDev = VisionConstants.linearStdDevBaseline * stDevFactor;
                angularStdDev = VisionConstants.angularStdDevBaseline * stDevFactor;

                if (cameraIndex < VisionConstants.cameraStdDevFactors.length){
                    linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                }
                } else {
                    linearStdDev = VisionConstants.linearStdDevBaseline;
                    angularStdDev = VisionConstants.angularStdDevBaseline;
                }

                drivetrain.addVisionMeasurement(observation.estPose().estimatedPose.toPose2d(),
                observation.estPose().timestampSeconds,
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/AcceptedEstimatedPoses",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/RejectedEstimatedPoses",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));  
        }
    }
}
