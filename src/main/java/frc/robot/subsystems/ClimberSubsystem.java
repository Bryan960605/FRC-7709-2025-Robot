// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final SparkMax climbMotor;

  private final SparkMaxConfig climbConfig;
  public ClimberSubsystem() {
    climbMotor = new SparkMax(ClimberConstants.climbMotor_ID, MotorType.kBrushless);

    climbConfig = new SparkMaxConfig();

    climbConfig.idleMode(IdleMode.kBrake);

    climbConfig.encoder.inverted(ClimberConstants.MotorReverse);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
