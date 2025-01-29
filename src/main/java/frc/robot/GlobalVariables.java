package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    
    public boolean extenderFull;
    public double speakerDistance;

    public DriverStation.Alliance alliance;

    private GlobalVariables() {
        extenderFull = false;
        alliance = null;
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}