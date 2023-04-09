package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CalibrateExtender extends CommandBase {
    private final Arm arm;

    /**
     * Calibrates the extender by retracting it slowly until it hits the limit
     * switch.
     * 
     * @param givenArm the arm subsystem
     */
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

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.getExtenderCalibration();
    }
}
