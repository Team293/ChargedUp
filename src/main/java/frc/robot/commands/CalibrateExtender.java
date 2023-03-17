package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CalibrateExtender extends CommandBase {
    private final Arm arm;

    public CalibrateExtender(Arm givenArm) {
        arm = givenArm;
        
        addRequirements(arm);                                                            
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Invalidate the extender calibrations.
        arm.invalidateExtenderCalibration();
    }                      

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* Nothing to continuously do. (periodic will zero the arm for us) */ 
        /* isFinished will check if the arm is still calibrating and end when done */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (true);
    }
}
