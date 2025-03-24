package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    
    public DriverStation.Alliance alliance;
    public int alignStatus;

    private GlobalVariables() {
        alliance = null;
        alignStatus = 0;
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}