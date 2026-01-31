package frc.robot.subsystems.shooter;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShooterIOReal implements ShooterIO {

    public ShooterIOReal(AbstractDriveTrainSimulation driveSim) {
    }

    @Override
    public void launchFuel(LinearVelocity velocityMps, Angle angleRad, Runnable onHit) {
       
    }
}