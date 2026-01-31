package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
// Over bumper Intake

import edu.wpi.first.units.measure.Distance;
public class FuelIntakeConstants {
    public static final int FUEL_INTAKE_CAPATICY = 15; // Top kapasitesi (max100)
    public static final IntakeSide FUEL_INTAKE_SIDE = IntakeSimulation.IntakeSide.FRONT; // Intake Tarafı 

    public static final Distance FUEL_INTAKE_WIDTH = Meters.of(0.3); // Intake genişlik
    public static final Distance FUEL_INTAKE_LENGTH = Meters.of(0.3); // Intake yukseklik
}
