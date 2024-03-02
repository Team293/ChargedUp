package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  private DoubleSolenoid climbSolenoid;
  private Compressor compressor;

  public climber() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);

    climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Put methods for controlling this subsystem here. Call these from Commands.
  public void climberDown() {
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void climberUp() {
    climbSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void stop() {
    climbSolenoid.set(DoubleSolenoid.Value.kOff);
  }
}
