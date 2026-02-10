// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    // Motor CAN ID'leri
    public static final int kLeftFrontID = 11;
    public static final int kLeftBackID = 12;//12
    public static final int kRightFrontID = 13; 
    public static final int kRightBackID = 14; // 4 yerine tahmin ediyorum, SparkMax hatasına göre düzenlenmeli

    // Gyro
    public static final int kPigeonID = 15;

    // --- FİZİKSEL ÖLÇÜMLER (MUTLAKA DÜZENLE) ---
    // İki teker arası mesafe (Track Width) - Metre cinsinden ölç!
    public static final double kTrackWidthMeters = Units.inchesToMeters(24.0); 
    
    // Tekerlek Yarıçapı (Radius) - Metre cinsinden
    // 6 inç tekerlek için yarıçap 3 inçtir.
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0); 

    // Sanziman Orani (Gear Ratio)
    // Motorun bir tur tekerleği döndürmesi için kaç tur atması gerektiği (Kit of Parts chassis genelde 10.71 veya 8.45 olur)
    public static final double kGearRatio = 10.71;


    // --- KONTROL SABİTLERİ (SYSID İLE BULUNMASI ÖNERİLİR) ---
    // Tahmini başlangıç değerleri:
    public static final double kS = 0.22; // Statik Sürtünme (Robotu kıpırdatan min voltaj)
    public static final double kV = 2.5;  // Hız sabiti (12 Volt / Max Hız formülünden yaklaşık hesap)
    public static final double kA = 0.4;  // İvme sabiti
    
    public static final double kP = 2.0;  // Hatayı düzeltme gücü (Otonomda robot titrerse bunu azalt)
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxSpeedMetersPerSecond = 4.0; // Robotun max hızı (m/s)
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5; // Max ivmelenme

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

  }
  
  public static class ShooterConstants {
    public static final int kShooterKrakenID = 22;
    // Tek motorlu Kraken Shooter
    
    public static final double kShooterSpeed = 0.3; // %50 güç
  }

  public static class ConveyorConstants {
    public static final int kConveyorSparkID = 40;
    public static final double kConveyorSpeed = 0.5;
  }

  public static class VisionConstants {
    public static final String kCameraName = "MEDCAM"; // PhotonVision arayüzündeki kamera adını buraya yazın
    
    // Hizalama PID Değerleri
    public static final double kTurnP = 0.05;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
    
    public static final double kTargetYaw = 0.0; // Hedeflenen açı (tam karşı)
  }
}
