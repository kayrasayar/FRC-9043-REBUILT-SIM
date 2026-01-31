package frc.robot.subsystems.vision.interfaces;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.vision.enums.AprilTagID;

public interface CameraInterface {
    
    void update();

    PhotonTrackedTarget getBestTarget();

    PhotonTrackedTarget getTarget(AprilTagID apriltag);

    boolean isSeen(AprilTagID aprilTag);

    List<PhotonTrackedTarget> getSeenTargets();

    PhotonPipelineResult getPipelineResult();
    
} 
