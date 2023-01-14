package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import static frc.robot.Constants.PneumaticConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climb extends SubsystemBase 
{
    
    private DoubleSolenoid climbSolenoid;

    private Compressor compressor;

    public Climb() {
        compressor = new Compressor(PNEUMATIC_MODULE_ID, PNUEMATIC_MODULE_TYPE);

        compressor.enableAnalog(LOWEST_COMPRESSOR_PSI, HIGHEST_COMPRESSOR_PSI);
        
        climbSolenoid = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNUEMATIC_MODULE_TYPE, CLIMB_RETRACTION_SOLENOID, CLIMB_EXTENSION_SOLENOID);  //  2 3

        addChild("lowPressureClimbSolenoid",climbSolenoid);

        //setting the climber to down position to prevent accidental deployment
        climberDown();
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public void climberDown() 
    {
        climbSolenoid.set(Value.kReverse);
    }

    public void climberUp() 
    {
        climbSolenoid.set(Value.kForward);
    }
}
