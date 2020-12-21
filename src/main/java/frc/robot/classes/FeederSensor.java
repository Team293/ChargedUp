/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;



import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class FeederSensor {
    public enum SensorState{
        LOW,
        RISING,
        HIGH,
        FALLING
    }

    private DigitalInput sensor;
    private SensorState state;

    public FeederSensor(DigitalInput newSensor){
        sensor = newSensor;
        if(sensor.get()) {
            state = SensorState.HIGH;
        } else {
            state = SensorState.LOW;
        }
    }

    public SensorState getState(){
        return state;
    }

    public SensorState update(){
        SensorState currState;
        if(sensor.get()) {
            currState = SensorState.HIGH;
        } else {
            currState = SensorState.LOW;
        }

        switch(state){
            case HIGH:
                if(currState == SensorState.LOW)
                {
                    state = SensorState.FALLING;
                }
                break;
            case LOW:
                if(currState == SensorState.HIGH)
                {
                    state = SensorState.RISING;
                }
                break;
            case FALLING:
                state = SensorState.LOW;
                break;
            case RISING:
                state = SensorState.HIGH;
                break;
            default:
                break;
        }

        return state;
    }
}
