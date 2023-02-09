package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroArm extends CommandBase {
    private final Arm arm;
    private boolean done;

    public ZeroArm(Arm givenArm) {
        arm = givenArm;
        done = false;
        
        addRequirements(arm);                                                            
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.startCalibration();
    }                      

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        done = arm.checkCalibration();
        if (done) {
            arm.zeroEncoders();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
