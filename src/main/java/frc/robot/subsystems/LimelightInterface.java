package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;

public class LimelightInterface extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final double TARGET_HEIGHT = 53.5d; // average height of april tags (they are on varying heights)
    private static final double LIMELIGHT_HEIGHT = 36d; // height of limelight mounted on robot
    private static final double LIMELIGHT_INITIAL_ANGLE = 17d; // in degrees the angle of the limelight mounted on robot
    private static final double VISION_ANGLE_TOLERANCE = 3.0d; // max deviation in actual angle measure

    private double tv; // target in sight
    private double tx; // horizontal offset from crosshair to target
    private double ty; // vertical offset from crosshair to target

    private double distanceToGoal; // in inches
    private int targetCount; // ensure target being identified correctly x amount of times before complete
    private Kinematics kinematics;

    public LimelightInterface(Kinematics kinematics) {
        this.kinematics = kinematics;
        

        tv = 0.0d;
        tx = 0.0d;
        ty = 0.0d;
        distanceToGoal = 0.0d;
        targetCount = 0;
    }

    @Override
    public void periodic() {
        tv = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("ty").getDouble(0);
        

        calcDist();

        if (Math.abs(tx) <= VISION_ANGLE_TOLERANCE) {
            targetCount++;
        } else {
            targetCount = 0;
        }
    }

    private void calcDist() {
        distanceToGoal = ((TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.cos(Math.toRadians(ty + LIMELIGHT_INITIAL_ANGLE))) * 12.0;
    }

    public double calcDistance() {
        return Math.sqrt((
            Math.pow((LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME)[0] - LimelightHelpers.getBotPose(LIMELIGHT_NAME)[0]), 2)) + 
            Math.pow(LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME)[1], 2)
            );
    }

    public double getDistance() {
        return distanceToGoal;
    }

    public boolean tooClose() {
        return distanceToGoal <= 24.0d;
    }

    public boolean tooFar() {
        return distanceToGoal >= 1.0;
    }

    public double getId() {
        return LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    }

    public boolean hasTarget() {
        return tv > 0;
    }

    public boolean isAligned() {
        return hasTarget() && Math.abs(tx) <= VISION_ANGLE_TOLERANCE && targetCount >= 7;
    }

}