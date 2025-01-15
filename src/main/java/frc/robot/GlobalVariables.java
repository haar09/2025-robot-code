package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    
    public boolean extenderFull;
    public double speakerDistance;

    public DriverStation.Alliance alliance;
    public HashMap<String, Boolean> detectedMap;

    private GlobalVariables() {
        detectedMap = new HashMap<>();
        extenderFull = false;
        speakerDistance = 0;
        alliance = null;
    }

    public void isDetected(boolean detected, String cameraName) {
        detectedMap.put(cameraName, detected);
        SmartDashboard.putBoolean("Camera Target", detectedMap.values().stream().anyMatch(Boolean::booleanValue));
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}