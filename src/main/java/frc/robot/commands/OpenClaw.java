package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrippyGravityClaw;

public class OpenClaw extends CommandBase{
  private final GrippyGravityClaw m_claw;

  public OpenClaw(GrippyGravityClaw subsystem){
    m_claw = subsystem;
    addRequirements(m_claw);
  }
  @Override
  public void initialize(){
    m_claw.spikeSolenoidPHForward();
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
