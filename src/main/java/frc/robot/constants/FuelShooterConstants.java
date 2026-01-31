package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class FuelShooterConstants {
    public static final Translation2d FUEL_SHOOTER_POSITION = new Translation2d(0.2, 0.2); // Shooter'ın robot üzerindeki ofseti (metre)
    public final static Distance FUEL_SHOOTER_HEIGHT = Meters.of(0.2);  // Fırlatılış yüksekliği
        
    public static final LinearVelocity FUEL_SHOOTER_SPEED = MetersPerSecond.of(7.5); // Fırlatılış hızı m/s

    public static final Angle FUEL_SHOOTER_ANGLE = Degrees.of(60); // Fırlatılış açısı derece
    
    public static final double FUEL_SHOOTER_TIME = 0.25; // Saniyede 4 top = her 0.25 saniyede bir atış (1 / 4 = 0.25)
}
