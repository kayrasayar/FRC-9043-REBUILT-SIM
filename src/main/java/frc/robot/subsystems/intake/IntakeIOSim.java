package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.constants.FuelIntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSim;

    public IntakeIOSim(SwerveDriveSimulation driveSim) {

        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSim,
            FuelIntakeConstants.FUEL_INTAKE_WIDTH,
            FuelIntakeConstants.FUEL_INTAKE_LENGTH, 
            FuelIntakeConstants.FUEL_INTAKE_SIDE,
            FuelIntakeConstants.FUEL_INTAKE_CAPATICY 
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