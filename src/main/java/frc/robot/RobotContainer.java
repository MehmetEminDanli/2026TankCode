// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; // Komut grupları için gerekli
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystems m_robotDrive = new DriveTrainSubsystems();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Auto Chooser yapılandırması - Hata korumalı
    if (com.pathplanner.lib.auto.AutoBuilder.isConfigured()) {
        autoChooser = com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser();
    } else {
        // Eğer PathPlanner ayarlanmadıysa boş bir menü oluştur, böylece robot çökmez
        autoChooser = new edu.wpi.first.wpilibj.smartdashboard.SendableChooser<>();
        autoChooser.setDefaultOption("PathPlanner Ayarlanmadı!", new edu.wpi.first.wpilibj2.command.InstantCommand());
    }
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Varsayılan sürüş komutu (Arcade Drive)
    m_robotDrive.setDefaultCommand(
        m_robotDrive.run(() -> m_robotDrive.arcadeDrive(
            -m_driverController.getLeftY(), 
            -m_driverController.getRawAxis(0)
        ))
    );

    // Shooter Kontrolü: 6. Buton (Xbox Right Bumper / RB)
    // Basılı tutulduğunda çalışır, bırakıldığında durur.
    m_driverController.button(1).whileTrue(m_shooterSubsystem.runShooterReverseCommand());//alma

    // BUTON 20: Shooter Hemen Başlar, 3 Saniye Sonra Conveyor Başlar
    // İkisi de tuş bırakılınca durur. ATIŞ İÇERİDEN ALMA
    m_driverController.button(5).whileTrue(
        Commands.parallel(
            m_shooterSubsystem.runShooterCommand(-0.8),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                m_conveyorSubsystem.runConveyorCommand(0.5)
            )
        )
    );

    // BUTON 21: Tam Ters Yön (Shooter Geri + 3sn Bekle + Conveyor Geri)
    m_driverController.button(6).whileTrue(
        Commands.parallel(
            m_shooterSubsystem.runShooterCommand(-0.5),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                m_conveyorSubsystem.runConveyorCommand(-0.3)
            )
        )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
