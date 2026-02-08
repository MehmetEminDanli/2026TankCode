// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystems extends SubsystemBase {
  private final SparkMax m_leftLeader = new SparkMax(DriveConstants.kLeftFrontID, MotorType.kBrushless);
  private final SparkMax m_leftFollower = new SparkMax(DriveConstants.kLeftBackID, MotorType.kBrushless);
  private final SparkMax m_rightLeader = new SparkMax(DriveConstants.kRightFrontID, MotorType.kBrushless);
  private final SparkMax m_rightFollower = new SparkMax(DriveConstants.kRightBackID, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonID);
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  
  private final DifferentialDriveOdometry m_odometry;

  public DriveTrainSubsystems() {
    configureMotors();
    
    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    // Reset gyro on init needed? Maybe not, usually done in auto init or manually.
    
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), 
        m_leftEncoder.getPosition(), 
        m_rightEncoder.getPosition()
    );

    configurePathPlanner();
  }

  private void configureMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    
    // Encoder dönüşüm faktörleri (Meters)
    // 2 * PI * Radius / GearRatio
    double positionFactor = (Math.PI * 2 * DriveConstants.kWheelRadiusMeters) / DriveConstants.kGearRatio;
    double velocityFactor = positionFactor / 60.0;
    
    config.encoder.positionConversionFactor(positionFactor);
    config.encoder.velocityConversionFactor(velocityFactor);

    m_leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Follower Config
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(m_leftLeader);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(m_rightLeader);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Sağ tarafı ters çevir (DifferentialDrive genellikle halleder ama config seviyesinde yapmak daha iyidir, 
    // ama DifferentialDrive kullanıyorsak bazen setInverted gerekebilir. 
    // Burada leader'ı invert ediyoruz.)
    m_rightLeader.setInverted(true); 
  }

  private void configurePathPlanner() {
      try {
          RobotConfig config = RobotConfig.fromGUISettings();

          AutoBuilder.configure(
              this::getPose, 
              this::resetPose, 
              this::getChassisSpeeds, 
              (speeds, feedforwards) -> driveRobotRelative(speeds), 
              new PPLTVController(0.02),
              config,
              () -> {
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
              },
              this
          );
      } catch (Exception e) {
          e.printStackTrace();
      }
  }

  @Override
  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(), 
        m_leftEncoder.getPosition(), 
        m_rightEncoder.getPosition()
    );
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getVelocity(), 
        m_rightEncoder.getVelocity()
    ));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);
    
    // Basit P kontrol veya Feedforward eklenebilir.
    // Simdilik voltaj yerine set ile suruyoruz. Idealde PIDController ile setVoltage yapilmali.
    double left = wheelSpeeds.leftMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    m_leftLeader.set(left);
    m_rightLeader.set(right);
    m_drive.feed();
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
}


