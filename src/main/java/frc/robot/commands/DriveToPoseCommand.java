package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystems;

/**
 * Robotu belirli bir (X, Y) konumuna ve (Rotation) açısına gitmesini sağlayan komut.
 * Basit P kontrolü kullanır: Önce döner, sonra gider, sonra son açıyı alır veya
 * eş zamanlı (Arcade) kontrol yapar.
 */
public class DriveToPoseCommand extends Command {
  private final DriveTrainSubsystems m_drive;
  private final Pose2d m_targetPose;
  
  private final PIDController m_driveController;
  private final PIDController m_turnController;

  public DriveToPoseCommand(DriveTrainSubsystems drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;

    // Uzaklık için PID (P: mesafe hatasına göre hız)
    m_driveController = new PIDController(1.0, 0.0, 0.0);
    
    // Dönüş için PID (P: açı hatasına göre dönüş hızı)
    m_turnController = new PIDController(0.05, 0.0, 0.0);
    m_turnController.enableContinuousInput(-180, 180);

    // Toleranslar (Hata payı)
    m_driveController.setTolerance(0.05); // 5 cm tolerans
    m_turnController.setTolerance(2.0);   // 2 derece tolerans

    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    
    // Hedefe olan mesafe
    double currentDistance = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    
    // Hedefe olan açı (Robotun burnu nereye bakmalı?)
    double angleToTargetDeg = Math.toDegrees(Math.atan2(
        m_targetPose.getY() - currentPose.getY(),
        m_targetPose.getX() - currentPose.getX()
    ));

    // Robotun şu anki açısı
    double currentAngleDeg = currentPose.getRotation().getDegrees();

    // 1. Önce hedefe dön veya giderken dön (Basit Arcade Drive mantığı)
    // Mesafeyi (Forward Speed) hesapla
    double forwardSpeed = m_driveController.calculate(-currentDistance, 0); 
    // Not: Mesafe her zaman pozitif geleceği için eksi ile çarptık ki "hata" 0 - distance olsun.
    // Ancak PID mantığında setpoint 0, measurement distance ise output negatif olur (hız ileri).
    // Düzeltme: Setpoint=0, Measurement=Distance -> Error = -Distance. KP * Error = -Speed.
    // Robot ileri gitmeli. Bu yüzden -output kullanacağız veya range ters çevireceğiz.
    // Şimdilik:
    forwardSpeed = m_driveController.calculate(currentDistance, 0); 
    // Eğer measurement=distance ve setpoint=0 ise, error = -distance.
    // KP=1 ise output = -distance.
    // Uzaklaştıkça negatif hız veriyor? Negatif hız robotu geri götürür mü?
    // Tank drive'da ArcadeDrive +X ileri ise pozitif olmalı.
    // Demek ki measurement=0, setpoint=distance dersek error=distance, output=positive.
    forwardSpeed = m_driveController.calculate(0, currentDistance);

    // Maksimum hızı sınırla
    forwardSpeed = Math.max(-0.5, Math.min(0.5, forwardSpeed));

    // Dönüş Hızı (Rotation Speed)
    // Şimdilik sadece hedefe bakarak gidelim, son duruş açısını (final rotation) sonra düşünürüz.
    // Hedefe giderken hedefe bakmalı.
    double rotationSpeed = m_turnController.calculate(currentAngleDeg, angleToTargetDeg);
    
    // Maksimum dönüş hızı sınırla
    rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));

    // Eğer çok yaklaştıysak (örn 10cm), artık son yönelim açısına (Header) dönelim
    if (currentDistance < 0.10) {
        forwardSpeed = 0;
        double targetHeading = m_targetPose.getRotation().getDegrees();
        rotationSpeed = m_turnController.calculate(currentAngleDeg, targetHeading);
    }
    
    // Arcade Drive ile sür
    // fwd: İleri hız, rot: Dönüş hızı
    m_drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_driveController.atSetpoint() && m_turnController.atSetpoint();
  }
}
