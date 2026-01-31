package frc.robot.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private float score = 0;
  private float shootCount = 0;
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void incrementsScore(){
    score++;
  }

  @Override
  public void periodic(){
    DogLog.log("Shooter/Succsesful Score",score);
    if(score>0){
      DogLog.log("Shooter/xG",score/shootCount);
    }
  }

  // Fırlatma komutu
  public void launchFuel(LinearVelocity velocityMps, Angle angleRad) {
    io.launchFuel(velocityMps, angleRad, this::incrementsScore);
    shootCount++;
  }
}