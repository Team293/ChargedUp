/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class FeederCounter {
    
    DigitalInput feederSensor;

    boolean prevSensorVal = false;

    public FeederCounter(DigitalInput sensor){
        feederSensor = sensor;
    }

    public boolean hasSwitchClosed(DigitalInput sensor){
        
        
        if(sensor.get() == true && prevSensorVal == false){
            return true;
        }
        prevSensorVal = sensor.get();
        
        return false;
    } 
    
}
