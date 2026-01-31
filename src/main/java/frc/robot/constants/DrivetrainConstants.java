package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class DrivetrainConstants {

  public static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = DriveTrainSimulationConfig.Default()
  .withBumperSize(Inches.of(0.3), Inches.of(0.3))
  .withRobotMass(Kilograms.of(51))
  .withGyro(COTS.ofNav2X())
  .withSwerveModule(COTS.ofThriftySwerve(
    DCMotor.getNEO(1), 
    DCMotor.getNEO(1), 
    1.19, 
    6
  ));

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(Meters.of(+0.3310000009), Meters.of(+0.2679999999)),
    new Translation2d(Meters.of(+0.3310000009), Meters.of(-0.2679999999)),
    new Translation2d(Meters.of(-0.3310000009), Meters.of(+0.2679999999)),
    new Translation2d(Meters.of(-0.3310000009), Meters.of(-0.2679999999))
  ); 
}
