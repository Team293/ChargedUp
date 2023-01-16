public class PistonMotor extands CommandBase{
  private final GrippyGravityClaw m_piston;

  public OpenClaw(GrippyGravityClaw subsystem){
    m_piston = subsystem;
    addRequirements(m_piston);
  }
  @Override
  public void initialize(){
    m_piston.PISTON_LEAD_SCREW_MOTOR();
  }

  public void holdingCube(){
    leadScrewMotor.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_IntegralZone(POSITION_PID_SLOT_ID, 1000);
    leadScrewMotor.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);
    }

  public void holdingCone(){
    leadScrewMotor.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_IntegralZone(POSITION_PID_SLOT_ID, 500);
    leadScrewMotor.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);
  }

  public void holdingNothing(){
    leadScrewMotor.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
    leadScrewMotor.config_IntegralZone(POSITION_PID_SLOT_ID, 0);
    leadScrewMotor.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);
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