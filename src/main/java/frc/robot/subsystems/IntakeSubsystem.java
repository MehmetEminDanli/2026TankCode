// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.Constants.IntakeConstants;;

public class IntakeSubsystem extends SubsystemBase {
    SparkMax intakeM = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void intake() {
    intakeM.setVoltage(6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    public void runIntake(double speed) {
    intakeM.set(speed);
  }

  /**
   * Motorları durdurur.
   */
  public void stopIntake() {
    intakeM.set(0);
  }
  
  /**
   * Komut: Bu komut çalıştığı sürece (WhileTrue) shooter döner,
   * komut bitince (bırakılınca) durur.
   */
  public Command runIntakeCommand(double speed) {
      // start: çalıştır, end: durdur
      return this.startEnd(
          () -> runIntake(speed), 
          () -> stopIntake()
          
      );
  }
}
