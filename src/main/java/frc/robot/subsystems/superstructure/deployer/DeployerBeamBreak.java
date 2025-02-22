package frc.robot.subsystems.superstructure.deployer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeployerBeamBreak extends SubsystemBase{
  private final DigitalInput left_beamBreak, right_beamBreak;
  public boolean left_value = false;
  public boolean right_value = false;

  public DeployerBeamBreak() {
    left_beamBreak = new DigitalInput(4);
    right_beamBreak = new DigitalInput(5);
    SmartDashboard.putBoolean("left", false);
    SmartDashboard.putBoolean("right", false);
  }

  @Override
  public void periodic() {
      left_value = left_beamBreak.get();
      //left_value = SmartDashboard.getBoolean("left", false);
      Logger.recordOutput("Deployer/Beam Break/Left Beam", left_value);

      right_value = right_beamBreak.get();
      //right_value = SmartDashboard.getBoolean("right", false);
      Logger.recordOutput("Deployer/Beam Break/Right Beam", right_value);

      SmartDashboard.putBoolean("Deployer Ready", left_value && right_value);
  }   
}