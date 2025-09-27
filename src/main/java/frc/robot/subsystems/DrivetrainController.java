package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainController {
    private final PoseEstimator8736 poseEstimator;

    public DrivetrainController(PoseEstimator8736 poseEstimator) {
        this.poseEstimator = poseEstimator;
    }
    public ChassisSpeeds fieldToRobotChassisSpeeds(ChassisSpeeds fieldOriented)
    {
        double angle = 2*Math.PI*poseEstimator.getYaw();

        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);

        double vxRobot = fieldOriented.vxMetersPerSecond*cosA + fieldOriented.vyMetersPerSecond*sinA;
        double vyRobot = -fieldOriented.vxMetersPerSecond*sinA + fieldOriented.vyMetersPerSecond*cosA;
        
        return new ChassisSpeeds(vxRobot, vyRobot, fieldOriented.omegaRadiansPerSecond);
    }
}