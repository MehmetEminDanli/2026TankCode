package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_krakenMotor;

  public ShooterSubsystem() {
    m_krakenMotor = new TalonFX(ShooterConstants.kShooterKrakenID);

    configureMotors();
  }

  private void configureMotors() {
    // Kraken (TalonFX - CTRE) Ayarları
    // Kraken için varsayılan configs genellikle yeterlidir, sadece Neutral modunu set ediyoruz.
    m_krakenMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Motorları belirli bir hızda çalıştırır.
   * @param speedHızı -1.0 ile 1.0 arasında bir değer.
   */
  public void runShooter(double speed) {
    m_krakenMotor.set(speed);
  }

  /**
   * Motorları durdurur.
   */
  public void stopShooter() {
    m_krakenMotor.set(0);
  }
  
  /**
   * Komut: Bu komut çalıştığı sürece (WhileTrue) shooter döner,
   * komut bitince (bırakılınca) durur.
   */
  public Command runShooterCommand(double speed) {
      // start: çalıştır, end: durdur
      return this.startEnd(
          () -> runShooter(speed), 
          () -> stopShooter()
      );
  }

  public Command runShooterReverseCommand() {
      // start: geriye çalıştır, end: durdur
      return this.startEnd(
          () -> runShooter(-ShooterConstants.kShooterSpeed), 
          () -> stopShooter()
      );
  }
}
