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
    public static final int kLeftFrontID = 1;
    public static final int kLeftBackID = 2;
    public static final int kRightFrontID = 3;
    public static final int kRightBackID = 4;

    // Gyro
    public static final int kPigeonID = 0;

    // PID ve Feedforward Constants
    public static final double kS = 0.1;
    public static final double kV = 0.1; 
    public static final double kA = 0.1;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kTrackWidthMeters = Units.inchesToMeters(24.0);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;

    public static final double kGearRatio = 10.71;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
  }
}
