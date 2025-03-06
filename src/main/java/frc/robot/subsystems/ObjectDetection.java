package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class ObjectDetection extends SubsystemBase{
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult result;
    private double tx, ratio;
    private boolean hasTargets;
    private final Alert disconnectedAlert;

    public ObjectDetection() {        
        camera = new PhotonCamera("ar0234");

        hasTargets = false;
        tx = 0;
        ratio = 0;

        disconnectedAlert = new Alert("Object Detection is disconnected.", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic(){
        results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
        result = results.get(results.size() -1);
        hasTargets = result.hasTargets();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            var dimensions = getTargetDimensions(target);
            ratio = dimensions[0] / dimensions[1];
            tx = result.getBestTarget().getYaw();
        } else {
            tx = 0;
        }
        } else {
            hasTargets = false;
            tx = 0;
        }
        Logger.recordOutput("Vision/ar0234/Target Valid", hasTargets);
        Logger.recordOutput("Vision/ar0234/Object TX", tx);
        Logger.recordOutput("Vision/ar0234/Connected", camera.isConnected());
        disconnectedAlert.set(!camera.isConnected());
    }

    public double getTX() {
        return tx;
    }

    public double getRatio() {
        return ratio;
    }

    public boolean targetValid() {
       return hasTargets;
    }

    // Calculate width and height of the minimum area rectangle from corners
    public static double[] getTargetDimensions(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        
        if (corners.size() != 4) {
            // Handle invalid corner count
            return new double[] {0, 0};
        }

        // Sort corners to find min/max x and y
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (TargetCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        // For a rotated rectangle, this is an approximation
        // For precise dimensions, we'd need to know the rotation angle
        double width = maxX - minX;
        double height = maxY - minY;
        
        return new double[] {width, height};
    }
}
