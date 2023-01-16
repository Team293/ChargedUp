package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrippyGravityClaw extends SubsystemBase {
  private DoubleSolenoid spikeSolenoidPH;
  private Compressor pcmCompressor;

  public GrippyGravityClaw() {
    spikeSolenoidPH = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);
    pcmCompressor = new Compressor(PNEUMATIC_MODULE_ID, PneumaticsModuleType.REVPH);

    pcmCompressor.enableAnalog(LOWEST_COMPRESSOR_PSI, HIGHEST_COMPRESSOR_PSI);
  }

  public void spikeSolenoidPHForward(){
    spikeSolenoidPH.set(Value.kForward);
  }
  public void spikeSolenoidPHReverse(){
    spikeSolenoidPH.set(Value.kReverse);
  }
}
