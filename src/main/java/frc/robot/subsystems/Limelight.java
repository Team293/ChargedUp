package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SpikeBoard;

public class Limelight extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";

    private double m_tx;
    private double m_ty;
    private double distanceToTag;
    private double distanceToTagX;
    private double distanceToTagY;
    private boolean targetFound;
    private double tagId;

    private final LimelightHelpers m_limelight; 
    private static SpikeBoard limelightTab;
    private final Kinematics m_kinematics;

    public Limelight(LimelightHelpers limelightHelper, Kinematics kinematics) {
        m_limelight = limelightHelper;
        m_kinematics = kinematics;
    }

    @Override
    public void periodic() {
        targetFound = LimelightHelpers.getTV(LIMELIGHT_NAME);
        m_tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        m_ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        distanceToTag = getDistance();
        distanceToTagX = getDistanceX();
        distanceToTagY = getDistanceY();
        tagId = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        getTab().setBoolean("April tag detected", targetFound);
        getTab().setDouble("tx", m_tx);
        getTab().setDouble("ty", m_ty);
        getTab().setDouble("Distance to tag", distanceToTag);
        getTab().setDouble("Distance to tag X", distanceToTagX);
        getTab().setDouble("Distance to tag Y", distanceToTagY);
    }

    public static SpikeBoard getTab() {
        if (limelightTab == null) {
            limelightTab = new SpikeBoard("Limelight");
        }
        return limelightTab;
    }

    public LimelightHelpers getHelper() {
        return m_limelight;
    }
    
    public String getLimelightName() {
        return LIMELIGHT_NAME;
    }

    public double getDistance() { // Distance from the robot to the target (includes x and y) (in feet)
        double[] tagPositionRaw = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME);
        Position2D tagPosition = new Position2D(tagPositionRaw[0], tagPositionRaw[1], tagPositionRaw[5]); // x, y, rz (also robot space)
        Position2D robotPosition = m_kinematics.getPose(); // Robot space
        return Math.sqrt(
            Math.pow(tagPosition.getX() - robotPosition.getX(), 2) +
            Math.pow(tagPosition.getY() - robotPosition.getY(), 2));
    }

    public double getDistanceX() { // (in feet)
        double[] tagPositionRaw = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME);
        Position2D tagPosition = new Position2D(tagPositionRaw[0], tagPositionRaw[1], tagPositionRaw[5]); // x, y, rz (also robot space)
        Position2D robotPosition = m_kinematics.getPose(); // Robot space
        return tagPosition.getX() - robotPosition.getX();
    }

    public double getDistanceY() { // (in feet)
        double[] tagPositionRaw = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME);
        Position2D tagPosition = new Position2D(tagPositionRaw[0], tagPositionRaw[1], tagPositionRaw[5]); // x, y, rz (also robot space)
        Position2D robotPosition = m_kinematics.getPose(); // Robot space
        return tagPosition.getY() - robotPosition.getY();
    }

    public double getAngle() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    public boolean isAligned() {
        return Math.abs(getAngle()) <= m_kinematics.getPose().getHeadingDegrees() + 3  && getDistance() < 0.5;
    }

    public double getTagId() {
        return tagId;
    }
}
