package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class GrippyGravityClaw extends SubsystemBase {
  private DoubleSolenoid spikeSolenoidPH;
  private Compressor pcmCompressor;
  private final XboxController m_xboxcontroller;
  private WPI_TalonFX leadScrewMotor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
 
  public GrippyGravityClaw() {
    spikeSolenoidPH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);
    leadScrewMotor = new WPI_TalonFX(PISTON_LEAD_SCREW_MOTOR);
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Yellow", detectedColor.yellow);
    SmartDashboard.putNumber("Purple", detectedColor.purple);
    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    pcmCompressor.enableAnalog(LOWEST_COMPRESSOR_PSI, HIGHEST_COMPRESSOR_PSI);
  }

  public void spikeSolenoidPHForward(){
    spikeSolenoidPH.set(Value.kForward);
  }
  public void spikeSolenoidPHReverse(){
    spikeSolenoidPH.set(Value.kReverse);
  }
}
