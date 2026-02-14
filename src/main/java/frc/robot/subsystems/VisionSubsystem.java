package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonPoseEstimator m_poseEstimator;
  private AprilTagFieldLayout m_fieldLayout;

  public VisionSubsystem() {
    // Yapıcı metod: Kamera ismini kullanarak PhotonCamera oluşturur
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
    
    // AprilTag Field Layout yükleme
    try {
        // En güvenli yükleme yöntemi (2024+)
        m_fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        
        // Robot - Kamera Dönüşümü (Transform)
        Transform3d robotToCam = new Transform3d(
            new Translation3d(VisionConstants.kCamX, VisionConstants.kCamY, VisionConstants.kCamZ),
            new Rotation3d(VisionConstants.kCamRoll, VisionConstants.kCamPitch, VisionConstants.kCamYaw)
        );
        
        m_poseEstimator = new PhotonPoseEstimator(
            m_fieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            robotToCam
        );
        SmartDashboard.putBoolean("Vision Initialized", true);
        
    } catch (Exception e) {
        DriverStation.reportError("AprilTag Field Layout yüklenemedi: " + e.getMessage(), true);
        SmartDashboard.putString("Vision Init Error", e.toString());
        SmartDashboard.putBoolean("Vision Initialized", false);
    }
  }

  /**
   * Tahmini robot pozisyonunu döndürür.
   * @return Optional<EstimatedRobotPose> - Eğer target varsa ve pose hesaplanabiliyorsa
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (m_poseEstimator == null) return Optional.empty();
    return m_poseEstimator.update(m_camera.getLatestResult());
  }


  @Override
  public void periodic() {
    // SmartDashboard'a bilgi yazdırma (Debug için)
    PhotonPipelineResult result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("Target Found", result.hasTargets());
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      SmartDashboard.putNumber("Target Yaw", bestTarget.getYaw());
      int targetId = bestTarget.getFiducialId();
      SmartDashboard.putNumber("Target ID", targetId);
      
      // Check if tag exists in layout
      if (m_fieldLayout != null) {
          boolean inLayout = m_fieldLayout.getTagPose(targetId).isPresent();
          SmartDashboard.putBoolean("Tag In Layout", inLayout);
          
          double ambiguity = bestTarget.getPoseAmbiguity();
          SmartDashboard.putNumber("Best Ambiguity", ambiguity);

          Transform3d camToTarget = bestTarget.getBestCameraToTarget();
          SmartDashboard.putString("Cam->Target", camToTarget.toString());

          if (ambiguity == -1.0) {
             SmartDashboard.putString("Vision Status", "HATALI: 3D Veri Yok! Kamera kalibre edilmemiş.");
          } else if (!inLayout) {
             SmartDashboard.putString("Vision Status", "Tag ID " + targetId + " layout içinde yok!");
          } else if (ambiguity > 0.2) {
             SmartDashboard.putString("Vision Status", "Ambiguity çok yüksek (>0.2): " + ambiguity);
          } else {
             SmartDashboard.putString("Vision Status", "Tag Geçerli - Hesaplama Bekleniyor");
          }
      }

      if (m_poseEstimator != null) {
          // Force update to check validity in dashboard
          m_poseEstimator.update(result).ifPresentOrElse(pose -> {
              SmartDashboard.putString("Internal Vision Pose", pose.estimatedPose.toPose2d().toString());
              SmartDashboard.putBoolean("Estimator Has Pose", true);
          }, () -> {
              SmartDashboard.putBoolean("Estimator Has Pose", false);
          });
      }
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
