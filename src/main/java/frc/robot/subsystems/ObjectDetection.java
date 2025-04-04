//2.4 areadan sonra alignlı oluyor
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class ObjectDetection extends SubsystemBase{
    private final PhotonCamera camera;
    private double tx, area;
    private boolean hasTargets;
    private final Alert disconnectedAlert;

    public ObjectDetection() {        
        camera = new PhotonCamera("ar0234");

        hasTargets = false;
        tx = 0;
        area = 0;

        disconnectedAlert = new Alert("Object Detection is disconnected.", Alert.AlertType.kWarning);
    }

    private int cyclesToSkip = 0;

    @Override
    public void periodic(){

        if (cyclesToSkip > 0) {
            cyclesToSkip--;
            return;
        }

        var results = camera.getAllUnreadResults();
        hasTargets = false;
        tx = 0;
        area = 0;
        if (!results.isEmpty()){
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                    var target = result.getBestTarget();
                //if (target.getDetectedObjectClassID() == 1) {
                        tx = target.getYaw();
                        hasTargets = true;
                        area = target.getArea();
                        cyclesToSkip = 3;
                }
            }

        Logger.recordOutput("Vision/ar0234/Target Valid", hasTargets);
        Logger.recordOutput("Vision/ar0234/Object TX", tx);
        Logger.recordOutput("Vision/ar0234/Object Area", area);
        Logger.recordOutput("Vision/ar0234/Connected", camera.isConnected());
        disconnectedAlert.set(!camera.isConnected());
    }

    public double getTX() {
        return tx;
    }

    public double getArea() {
        return area;
    }

    public boolean targetValid() {
       return hasTargets;
    }
}
