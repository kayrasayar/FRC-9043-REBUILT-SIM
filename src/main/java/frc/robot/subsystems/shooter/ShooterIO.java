package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ShooterIO {
    void launchFuel(LinearVelocity velocityMps, Angle angleRad, Runnable onHit); // Topu fırlat
}
