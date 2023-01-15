package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrippyGravityClaw;

public class CloseClaw extends CommandBase{
  private final GrippyGravityClaw m_claw;

  public CloseClaw(GrippyGravityClaw subsystem){
    m_claw = subsystem;
    addRequirements(m_claw);
  }
  @Override
  public void initialize(){
    m_claw.spikeSolenoidPHReverse();
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return true;
  }

}
