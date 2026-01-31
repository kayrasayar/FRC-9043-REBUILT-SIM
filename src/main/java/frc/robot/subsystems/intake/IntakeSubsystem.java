package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  // Robotun dışından (komutlardan) çağrılacak fonksiyon
  public void setRunning(boolean run) {
    io.setRunning(run);
  }

  @Override
  public void periodic() {
    // Top sayısı
    DogLog.log("Intake/FuelCount", io.getGamePieceCount());
  }
  
  public boolean useFuel() {
        return io.useFuel();
  }
}