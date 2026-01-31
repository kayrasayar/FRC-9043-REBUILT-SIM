package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.enums.AprilTagID;
import frc.robot.subsystems.vision.interfaces.CameraInterface;

public class VisionProcessingSubsystem extends SubsystemBase {

  CameraInterface camera;

  PhotonPoseEstimator estimator;

  public VisionProcessingSubsystem(CameraInterface camera) {
    this.camera = camera;

    this.estimator = new PhotonPoseEstimator(
      FieldConstants.APRILTAG_FIELD_LAYOUT, 
      VisionConstants.ESTIMATION_STRATEGY, 
      VisionConstants.ROBOT_TO_CAMERA
    );
  }

  @Override
  public void periodic() {
    camera.update();
  }

  public double getBestTargetYaw() {
    if (getBestTarget() != null) {
        return getBestTarget().getYaw();
    }
    return 0.0;
  }

  public boolean canEstimatePose() {
    return estimator.update(camera.getPipelineResult()).isPresent();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return estimator.update(camera.getPipelineResult());
  }

  public boolean isSeen(AprilTagID id) {
    return camera.isSeen(id);
  }

  public PhotonTrackedTarget getTarget(AprilTagID id) {
    return camera.getTarget(id);
  }
  
  public Transform3d getTargetTransform() {
      var result = camera.getPipelineResult();
      if (result.hasTargets()) {
          // Hedefin kameraya göre 3D konumu
          return result.getBestTarget().getBestCameraToTarget();
      }
      return null;
  }

  public PhotonTrackedTarget getBestTarget() {
    return camera.getBestTarget();
  }

}
