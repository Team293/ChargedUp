package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;


import com.revrobotics.ColorSensorV3;

private final XboxController m_xboxcontroller;
private WPI_TalonFX leadScrewMotor;

public class ColorSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public ColorSensor(){
    leadScrewMotor = new WPI_TalonFX()
    
  }

 
}