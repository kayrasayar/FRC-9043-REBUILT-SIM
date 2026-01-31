package frc.robot.subsystems.vision.implementations;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.enums.AprilTagID;
import frc.robot.subsystems.vision.interfaces.CameraInterface;

public class SimCamera implements CameraInterface {

  PhotonCamera camera;

  PhotonCameraSim cameraSim;

  PhotonPipelineResult result;

  public SimCamera(VisionSystemSim visionSystemSim, String name) {
    this.camera = new PhotonCamera(name);
  
    this.cameraSim = new PhotonCameraSim(camera, VisionConstants.CAMERA_PROPERTIES);
    this.cameraSim.enableDrawWireframe(true);

    visionSystemSim.addAprilTags(FieldConstants.APRILTAG_FIELD_LAYOUT);
    visionSystemSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
  }

  @Override
  public void update() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.size() == 0) {
      this.result = new PhotonPipelineResult();
      return;
    }

    this.result = results.get(results.size() - 1);
  }

  @Override
  public PhotonTrackedTarget getBestTarget() {
    return this.result.getBestTarget();
  }

  @Override
  public PhotonTrackedTarget getTarget(AprilTagID aprilTagID) {
    for (PhotonTrackedTarget aprilTag : result.getTargets()) {
      if (aprilTagID.id == aprilTag.fiducialId) {
        return aprilTag;
      }
    }
    return null;
  }

  @Override
  public boolean isSeen(AprilTagID aprilTagID) {
    return this.getTarget(aprilTagID) == null;
  }

  @Override
  public List<PhotonTrackedTarget> getSeenTargets() {
    return result.getTargets();
  }

  @Override
  public PhotonPipelineResult getPipelineResult() {
    return result;
  }
  
}
