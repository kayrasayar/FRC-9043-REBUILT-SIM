// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.FuelShooterConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionProcessingSubsystem;
import frc.robot.subsystems.vision.implementations.SimCamera;

import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.simulation.VisionSystemSim;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final         CommandPS5Controller driverDualsense = new CommandPS5Controller(0);

    public VisionProcessingSubsystem visionSubsystem;
    private VisionSystemSim visionSim;


        // RobotContainer içine ekle
    private final PIDController xController = new PIDController(1.5, 0, 0); // İleri-Geri
    private final PIDController yController = new PIDController(1.5, 0, 0); // Sağ-Sol
    private final PIDController thetaController = new PIDController(0.05, 0, 0); // Dönüş

    // The robot's subsystems and commands are defined here...
    public final DrivetrainSubsystem       drivetrain  = new DrivetrainSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                  "swerve/neo"));

    private IntakeSubsystem intake = new IntakeSubsystem(
        Robot.isSimulation() ? new IntakeIOSim(drivetrain.getSwerveDriveSim()) : new IntakeIOReal(null)
    );

    private ShooterSubsystem shooter = new ShooterSubsystem(
          Robot.isSimulation() ? new ShooterIOSim(drivetrain.getSwerveDriveSim()) : new ShooterIOReal(null)
    );
    
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final PIDController aimController = new PIDController(0.05, 0, 0); // kP değerini test ederek artır/azalt

    private double sonAtisZamani = 0;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                                  () -> driverDualsense.getLeftY() * -1,
                                                                  () -> driverDualsense.getLeftX() * -1)
                                                              .withControllerRotationAxis(() -> driverDualsense.getRightX() * -1)
                                                              .deadband(ControllerConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(false);
  
    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverDualsense::getRightX,
                                                                                             driverDualsense::getRightY)
                                                             .headingWhile(true);
  
    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                               .allianceRelativeControl(false);
  
    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                                          () -> -driverDualsense.getLeftY(),
                                                                          () -> -driverDualsense.getLeftX())
                                                                      .withControllerRotationAxis(() -> driverDualsense.getRawAxis(
                                                                          2))
                                                                      .deadband(ControllerConstants.DEADBAND)
                                                                      .scaleTranslation(0.8)
                                                                      .allianceRelativeControl(false);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                                 .withControllerHeadingAxis(() ->
                                                                                                            Math.sin(
                                                                                                                driverDualsense.getRawAxis(
                                                                                                                    2) *
                                                                                                                Math.PI) *
                                                                                                            (Math.PI *
                                                                                                             2),
                                                                                                            () ->
                                                                                                            Math.cos(
                                                                                                                driverDualsense.getRawAxis(
                                                                                                                    2) *
                                                                                                                Math.PI) *
                                                                                                            (Math.PI *
                                                                                                             2))
                                                                                 .headingWhile(true)
                                                                                 .translationHeadingOffset(true)
                                                                                 .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                     0));
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
      
      DriverStation.silenceJoystickConnectionWarning(true);
  
      //Set the default auto (do nothing) 
      autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivetrain::zeroGyroWithAlliance)
                                                      .andThen(Commands.none()));
  
      //Add a simple auto option to have the robot drive forward for 1 second then stop
      autoChooser.addOption("Drive Forward", Commands.runOnce(drivetrain::zeroGyroWithAlliance).withTimeout(.2)
                                                  .andThen(drivetrain.driveForward().withTimeout(1)));
      //Put the autoChooser on the SmartDashboard
      SmartDashboard.putData("Auto Chooser", autoChooser);
  
      if (autoChooser.getSelected() == null ) {
      RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivetrain::zeroGyroWithAlliance));
      }
      
      if (Robot.isSimulation()) {
            // for simulation
            this.intake = new IntakeSubsystem(new IntakeIOSim(drivetrain.getSwerveDriveSim()));
            this.shooter = new ShooterSubsystem(new ShooterIOSim(drivetrain.getSwerveDriveSim()));
            this.visionSetup();
      } else {
          // for real robot
          this.intake = new IntakeSubsystem(new IntakeIOReal(null)); 
          this.shooter = new ShooterSubsystem(new ShooterIOReal(null));
      }
      
      // Default Command: Robot her zaman sürüş modunda olsun
      drivetrain.setDefaultCommand(drivetrain.driveFieldOriented(driveAngularVelocity));

      SimulatedArena.getInstance().resetFieldForAuto();
  
      // Configure the trigger bindings - Subsystemlar kurulduktan sonra en sonda olmalı!
      configureBindings();
  
  }

  public void visionSetup(){
// 'visionSim' artık bir yerel değişken değil, sınıfın değişkeni
    this.visionSim = new VisionSystemSim("GlobalVisionSim");
    
    SimCamera cameraSim = new SimCamera(visionSim, "MainCamera");
    this.visionSubsystem = new VisionProcessingSubsystem(cameraSim);
  }

  public Command getStalkerCommand() {
    return Commands.run(() -> {
        var transform = visionSubsystem.getTargetTransform();

        if (transform != null) {
            // Hedef: Tag'in 1.5 metre önünde durmak (X=1.5, Y=0, Açı=0)
            double xSpeed = xController.calculate(transform.getX(), 1.5);
            double ySpeed = yController.calculate(transform.getY(), 0);
            double rotSpeed = thetaController.calculate(transform.getRotation().getZ(), 0);

            // Hızları limitliyoruz (Robotun kontrolden çıkmaması için)
            xSpeed = MathUtil.clamp(xSpeed, -2, 10);
            ySpeed = MathUtil.clamp(ySpeed, -2, 10);

            // Robot-Relative sürüş (Çünkü veriler kameraya/robota göre geliyor)
            drivetrain.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        } else { // Hedef yoksa dur
        }
    }, drivetrain, visionSubsystem);
}
  public Command getAimingDriveCommand() {
      return Commands.run(() -> {
          // Sürücüden gelen X ve Y hızları
          double xSpeed = -driverDualsense.getLeftY() * RobotConstants.MAX_SPEED;
          double ySpeed = -driverDualsense.getLeftX() * RobotConstants.MAX_SPEED;
          
          double rotationSpeed;
          
          // Eğer bir AprilTag görüyorsak vizyon kontrolü ele alsın
          if (visionSubsystem != null && visionSubsystem.getBestTarget() != null) {
              double yaw = visionSubsystem.getBestTargetYaw();
              // PID: Hedef 0 derece (tam orta), mevcut açımız 'yaw'
              rotationSpeed = aimController.calculate(yaw, 0);
          } else {
              // Hedef yoksa manuel kontrole devam
              rotationSpeed = -driverDualsense.getRightX() * RobotConstants.MAX_SPEED;
          }

          // Drivetrain'i sür
          drivetrain.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
      }, drivetrain);
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivetrain.driveFieldOriented(driveAngularVelocityKeyboard);


    if (RobotBase.isSimulation())
    {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else
    {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivetrain.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                            ));
      driverDualsense.create().onTrue(Commands.runOnce(() -> drivetrain.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // driverDualsense.button(1).whileTrue(drivetrain.sysIdDriveMotorCommand());
      // driverDualsense.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                                () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      


    }
    if (DriverStation.isTest())
    {
     
    } else
    {
      driverDualsense.options().whileTrue((Commands.runOnce(drivetrain::zeroGyro)));
      driverDualsense.create().whileTrue(Commands.none());
      //driverDualsense.triangle().whileTrue(Commands.none());
      // driverDualsense.L1().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
      // driverDualsense.R1().onTrue(Commands.none());
      driverDualsense.L1().whileTrue(getAimingDriveCommand());
    }

    // intake commands 
    driverDualsense.cross().whileTrue(
        new RunCommand(() -> intake.setRunning(true), intake)
    ).onFalse(
        new RunCommand(() -> intake.setRunning(false), intake)
    );

    // Shooter commands
    // launchFuel(speed, angle) 
    driverDualsense.square().whileTrue(
        new RunCommand(() -> {
            double suan = Timer.getFPGATimestamp(); // FPGA'den anlık zamanı alıyoruz
            
            // Saniyede 4 top = her 0.25 saniyede bir atış (1 / 4 = 0.25)
            if (suan - sonAtisZamani >= FuelShooterConstants.FUEL_SHOOTER_TIME) {
                if (intake.useFuel()) { 
                    // Eğer mermi varsa fırlat
                    shooter.launchFuel(FuelShooterConstants.FUEL_SHOOTER_SPEED, FuelShooterConstants.FUEL_SHOOTER_ANGLE); 
                    sonAtisZamani = suan; // Zamanı güncelle
                } else {
                }
            }
        }, intake, shooter)
    );

  }

  public void periodic(){
    if (visionSim != null && drivetrain.getSwerveDriveSim() != null) {
        // 1. Vision sisteminden güncel tahmini konumu iste
        Pose2d realPhysicsPose = drivetrain.getSwerveDriveSim().getSimulatedDriveTrainPose();
        visionSim.update(realPhysicsPose);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivetrain.setMotorBrake(brake);
  }
}