package frc.robot.subsystems.intake.intakeRollers;

import edu.wpi.first.wpilibj.RobotController;

public class NoIntakeRollers implements IntakeRollersIO {
    private double appliedVoltage = 0;

    public NoIntakeRollers() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        appliedVoltage = percentage * RobotController.getBatteryVoltage();
    }

    @Override
    public void stop(){
        appliedVoltage = 0;
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
    }
}