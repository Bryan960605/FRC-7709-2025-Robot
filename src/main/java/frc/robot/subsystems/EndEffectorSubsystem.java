// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakewheel;
  private final TalonFX intakeArm;
  private final CANcoder armAbsolutedEncoder;
  private final DigitalInput irSensor;

  private final TalonFXConfiguration wheelConfig;
  private final TalonFXConfiguration armConfig;
  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController armPID;
  private final ArmFeedforward armFeedforward;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  private double arriveAngle;

  public EndEffectorSubsystem() {
    intakewheel = new TalonFX(EndEffectorConstants.intakeWheel_ID);
    intakeArm = new TalonFX(EndEffectorConstants.intakeArm_ID);
    armAbsolutedEncoder = new CANcoder(EndEffectorConstants.armAbsolutedEncoder_ID);
    irSensor = new DigitalInput(EndEffectorConstants.irSensor_ID);
    arriveAngle = EndEffectorConstants.primitiveAngle;

    // Motor Configurations
    wheelConfig = new TalonFXConfiguration();
    armConfig = new TalonFXConfiguration();

    wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakewheel.getConfigurator().apply(wheelConfig);
    intakeArm.getConfigurator().apply(armConfig);

    // Absolute Encoder Configurations
    absolutedEncoderConfig = new CANcoderConfiguration();
    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.absolutedEncoderOffset;
    armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID Controller and Feedforward
    armPID = new PIDController(EndEffectorConstants.armPID_Kp, EndEffectorConstants.armPID_Ki, EndEffectorConstants.armPID_Kd);
    armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
    armPID.setIntegratorRange(EndEffectorConstants.armPIDMinOutput, EndEffectorConstants.armPIDMaxOutput);
  }

  public void intakeCoral_Arm() {
    arriveAngle = EndEffectorConstants.coralStationAngle;
  }

  public void intakeCoral_Wheel() {
    intakewheel.set(EndEffectorConstants.coralInVol);
  }

  public void outCoral_L1_Arm() {
    arriveAngle = EndEffectorConstants.coralL1Angle;
  }

  public void outCoral_L1_Wheel() {
    intakewheel.set(EndEffectorConstants.coralL1OutVol);
  }

  public void outCoral_L2_Arm() {
    arriveAngle = EndEffectorConstants.coralL2Angle;
  }

  public void outCoral_L2_Wheel() {
    intakewheel.set(EndEffectorConstants.coralL2OutVol);
  }

  public void outCoral_L3_Arm() {
    arriveAngle = EndEffectorConstants.coralL3Angle;
  }

  public void outCoral_L3_Wheel() {
    intakewheel.set(EndEffectorConstants.coralL3OutVol);
  }

  public void outCoral_L4_Arm() {
    arriveAngle = EndEffectorConstants.coralL4Angle;
  }

  public void outCoral_L4_Wheel() {
    intakewheel.set(EndEffectorConstants.coralL4OutVol);
  }

  public void shootNet_Arm() {
    arriveAngle = EndEffectorConstants.algaeNetAngle;
  }

  public void shootNet_Wheel() {
    intakewheel.set(EndEffectorConstants.algaeShootNetVol);
  }

  public void shootProcessor_Arm() {
    arriveAngle = EndEffectorConstants.algaeProccesorAngle;
  }

  public void shootProcessor_Wheel() {
    intakewheel.set(EndEffectorConstants.algaeShootProcessorVol);
  }

  public void intakeAlgae_Low_Arm() {
    arriveAngle = EndEffectorConstants.algaeLowInAngle;
  }

  public void intakeAlgae_Low_Wheel() {
    intakewheel.set(EndEffectorConstants.algaeLowInVol);
  }

  public void intakeAlgae_High_Arm() {
    arriveAngle = EndEffectorConstants.algaeHighInAngle;
  }

  public void intakeAlgae_High_Wheel() {
    intakewheel.set(EndEffectorConstants.algaeHighInVol);
  }

  public void intakeAlgae_Floor_Arm() {
    arriveAngle = EndEffectorConstants.algaeFloorAngle;
  }

  public void intakeAlgae_Floor_Wheel() {
    intakewheel.set(EndEffectorConstants.algaeFloorInVol);
  }

  public void primitiveArm() {
    arriveAngle = EndEffectorConstants.primitiveAngle;
  }

  public void holdAlgae(){
    intakewheel.set(EndEffectorConstants.algaeHoldVol);
  }

  public void stopWheel() {
    intakewheel.set(0);
  }

  public double getAngle() {
    return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getRadians() {
    return Units.degreesToRadians(getAngle());
  }

  public double getVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(armAbsolutedEncoder.getVelocity().getValueAsDouble()*60);
  }

  public boolean hasGamePiece() {
    return !irSensor.get();
  }

  public boolean arriveSetPoint() {
    return (Math.abs(armPID.getError()) <= 1);
  }

  public void setMode(){

  }

  @Override
  public void periodic() {
    // Arm
    pidOutput = armPID.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getRadians(), getVelocity());
    // feedforwardOutput = armFeedforward.calculate(getAngle(), pidOutput);
    // if(feedforwardOutput >= EndEffectorConstants.feedForwardMax) {
    //   feedforwardOutput = EndEffectorConstants.feedForwardMax;
    // } 
    output = pidOutput + feedforwardOutput;
    intakeArm.set(output);

    //SmartDashboard
    SmartDashboard.putNumber("EndEffector/pidOutput", pidOutput);
    SmartDashboard.putNumber("EndEffector/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("EndEffector/Output", output);
    SmartDashboard.putBoolean("EndEffector/hasGamePiece", hasGamePiece());
  }
}
