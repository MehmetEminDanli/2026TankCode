// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystems;

public class JoystickDriveCommand extends Command {
  private final DriveTrainSubsystems m_driveTrain;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_turnSupplier;

  /** 
   * Joystick ile Arcade Drive kontrolü.
   * @param driveTrain DriveTrain Subsystem
   * @param speedSupplier İleri/Geri hız (Joystick Y ekseni)
   * @param turnSupplier Dönüş hızı (Joystick X veya Z ekseni)
   */
  public JoystickDriveCommand(DriveTrainSubsystems driveTrain, DoubleSupplier speedSupplier, DoubleSupplier turnSupplier) {
    m_driveTrain = driveTrain;
    m_speedSupplier = speedSupplier;
    m_turnSupplier = turnSupplier;
    
    // Subsystem'i kullanacağımızı sisteme bildiriyoruz (başkası kullanamaz)
    addRequirements(driveTrain);
  }

  // Komut başladığında ne olacak?
  @Override
  public void initialize() {
    System.out.println("Joystick Drive Started");
  }

  // Her döngüde (20ms'de bir) çalışacak kısım
  @Override
  public void execute() {
    // Joystick değerlerini doğrudan al
    double speed = m_speedSupplier.getAsDouble();
    double turn = m_turnSupplier.getAsDouble();

    // Robotu sür (Basit Arcade Drive)
    // Eğer robot ters gidiyorsa speed başına - koyun: -speed
    // Eğer dönmüyorsa turn ve speed yerini değiştirin test edin.
    m_driveTrain.arcadeDrive(speed, turn * 0.75); // Dönüşü biraz yavaşlattık (%75)
  }

  // Komut bittiğinde motorları durdur
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0);
  }

  // Bu komut varsayılan komut olduğu için asla bitmez
  @Override
  public boolean isFinished() {
    return false;
  }
}
