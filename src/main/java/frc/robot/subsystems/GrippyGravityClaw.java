package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrippyGravityClaw extends SubsystemBase {
  private DoubleSolenoid spikeSolenoidPH;
  private Compressor pcmCompressor;

  public GrippyGravityClaw() {
    spikeSolenoidPH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);

    pcmCompressor.enableAnalog(100.0, 119.5);
  }

  public void spikeSolenoidPHForward(){
    spikeSolenoidPH.set(Value.kForward);
  }
  public void spikeSolenoidPHReverse(){
    spikeSolenoidPH.set(Value.kOff);
  }

}
