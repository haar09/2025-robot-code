package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBeamBreak extends SubsystemBase{
  private final DigitalInput m_beamBreak, lower_beambreak;
  public boolean upper_value;
  public boolean lower_value;

  public IntakeBeamBreak() {
    m_beamBreak = new DigitalInput(0);
    lower_beambreak = new DigitalInput(2);
    SmartDashboard.putBoolean("upper", false);
    SmartDashboard.putBoolean("lower", false);
  }

  @Override
  public void periodic() {
      //upper_value = !m_beamBreak.get();
      upper_value = SmartDashboard.getBoolean("upper", false);
      Logger.recordOutput("Intake/Beam Break/Upper Beam", upper_value);

      //lower_value = !lower_beambreak.get();
      lower_value = SmartDashboard.getBoolean("lower", false);
      Logger.recordOutput("Intake/Beam Break/Lower Beam", lower_value);
  }   
}