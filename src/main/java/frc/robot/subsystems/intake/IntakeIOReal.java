package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOReal implements IntakeIO {
    private final IntakeSimulation intakeSim;

    public IntakeIOReal(AbstractDriveTrainSimulation driveSim) {
        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, Meters.of(0.6), Meters.of(0.2), 
            IntakeSimulation.IntakeSide.FRONT, 5 // Maks 5 top
        );
    }

    @Override
    public void setRunning(boolean run) {
        if (run) intakeSim.startIntake(); else intakeSim.stopIntake();
    }

    @Override
    public int getGamePieceCount() {
        return intakeSim.getGamePiecesAmount();
    }

    @Override
    public boolean useFuel(){
        return intakeSim.obtainGamePieceFromIntake();
    }
}