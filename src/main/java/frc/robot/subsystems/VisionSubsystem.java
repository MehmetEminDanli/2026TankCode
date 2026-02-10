package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;

  public VisionSubsystem() {
    // Yapıcı metod: Kamera ismini kullanarak PhotonCamera oluşturur
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  @Override
  public void periodic() {
    // SmartDashboard'a bilgi yazdırma (Debug için)
    PhotonPipelineResult result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("Target Found", result.hasTargets());
    if (result.hasTargets()) {
      SmartDashboard.putNumber("Target Yaw", result.getBestTarget().getYaw());
    }
  }

  /**
   * Kameradan gelen son sonucu döndürür.
   * @return PhotonPipelineResult
   */
  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  /**
   * En iyi (en büyük/en güvenilir) hedefi döndürür. 
   * @return PhotonTrackedTarget veya null
   */
  public PhotonTrackedTarget getBestTarget() {
    var result = getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  /**
   * Belirli ID'lere sahip hedefleri filtreleyerek en iyi hedefi döndürür.
   * @param targetIds Aranacak AprilTag ID'leri
   * @return Uygun hedef varsa PhotonTrackedTarget, yoksa null
   */
  public PhotonTrackedTarget getTargetWithId(int... targetIds) {
    var result = getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        int id = target.getFiducialId();
        for (int targetId : targetIds) {
          if (id == targetId) {
            return target;
          }
        }
      }
    }
    return null;
  }
}
