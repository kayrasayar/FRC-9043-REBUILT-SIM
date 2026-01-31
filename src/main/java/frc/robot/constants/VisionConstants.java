package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

  public static final PoseStrategy ESTIMATION_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
    Meters.of(0.20), 
    Meters.of(0), 
    Meters.of(0.18), 
    new Rotation3d(
      Degrees.of(0),      
      Degrees.of(-15),      
      Degrees.of(0)      
    )
  );
  
  public static SimCameraProperties CAMERA_PROPERTIES = new SimCameraProperties();

  public static final String CAMERA_NAME = "CAM";

  public static final String VISION_SYSTEM_NAME = "Main";

  static {
    CAMERA_PROPERTIES.setCalibration(640, 480, Rotation2d.fromDegrees(90));
    CAMERA_PROPERTIES.setCalibError(0.25, 0.08);
    CAMERA_PROPERTIES.setFPS(30);
    CAMERA_PROPERTIES.setAvgLatencyMs(50);
    CAMERA_PROPERTIES.setLatencyStdDevMs(10);
  }
}
