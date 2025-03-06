package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    
    public DriverStation.Alliance alliance;

    private GlobalVariables() {
        alliance = null;
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}