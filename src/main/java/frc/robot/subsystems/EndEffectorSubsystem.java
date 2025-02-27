// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.Mode;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakewheel;
  private final TalonFX intakeArm;
  private final CANcoder armAbsolutedEncoder;
  private final DigitalInput irSensor_Coral;
  private final DigitalInput irSensor_Algae;

  private final TalonFXConfiguration wheelConfig;
  private final TalonFXConfiguration armConfig;
  private final CANcoderConfiguration absolutedEncoderConfig;
  private final MotionMagicConfigs speedMotionMagicConfigs;
  private final MotionMagicConfigs turnMotionMagicConfigs;
  private final Slot0Configs speedSlot0Configs;
  private final Slot1Configs slot1Configs;
  private final MotionMagicVelocityVoltage request_EndEffectorSpeed;
  private final MotionMagicVoltage m_request;
  

  private final PIDController armPID;
  private ArmFeedforward armFeedforward;

  private final Timer timer_Coral;
  private final Timer timer_Algae;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  private double arriveAngle;
  private double nowTime_Coral;
  private double nowTime_Algae;
  private boolean shouldStart_Coral;
  private boolean shouldStart_Algae;
  private boolean shouldMotorTurn_HasItem;
  private boolean shouldMotorReset;
  private double nowPosition;
  
    public EndEffectorSubsystem() {
      intakewheel = new TalonFX(EndEffectorConstants.intakeWheel_ID);
      intakeArm = new TalonFX(EndEffectorConstants.intakeArm_ID);
      armAbsolutedEncoder = new CANcoder(EndEffectorConstants.armAbsolutedEncoder_ID);
      irSensor_Coral = new DigitalInput(EndEffectorConstants.irSensor_Coral_ID);
      irSensor_Algae = new DigitalInput(EndEffectorConstants.irSensor_Algae_ID);
      arriveAngle = EndEffectorConstants.primitiveAngle;
      timer_Coral = new Timer();
      timer_Algae = new Timer();
      shouldStart_Coral = true;
      shouldStart_Algae = true;
      m_request = new MotionMagicVoltage(0);
      slot1Configs = new Slot1Configs();
      turnMotionMagicConfigs = new MotionMagicConfigs();
  
      // Motor Configurations
      wheelConfig = new TalonFXConfiguration();
      armConfig = new TalonFXConfiguration();
      speedMotionMagicConfigs = new MotionMagicConfigs();
      speedSlot0Configs = wheelConfig.Slot0;
      request_EndEffectorSpeed = new MotionMagicVelocityVoltage(0);
  
      wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      
      //slot0
      speedSlot0Configs.kS = 0;
      speedSlot0Configs.kV = 0;
      speedSlot0Configs.kA = 0;
      speedSlot0Configs.kP = 0.3;
      speedSlot0Configs.kI = 0;
      speedSlot0Configs.kD = 0;
  
      slot1Configs.kG = 0;
      slot1Configs.kS = 0;
      slot1Configs.kV = 0;
      slot1Configs.kA = 0;
      slot1Configs.kP = 10;
      slot1Configs.kI = 0;
      slot1Configs.kD = 0;
      //MotioinMagic Config

      turnMotionMagicConfigs.MotionMagicCruiseVelocity = 40;
      turnMotionMagicConfigs.MotionMagicAcceleration = 80;
      turnMotionMagicConfigs.MotionMagicJerk = 400;

      speedMotionMagicConfigs.MotionMagicAcceleration = 400;
      speedMotionMagicConfigs.MotionMagicJerk = 4000;
      // Absolute Encoder Configurations
      absolutedEncoderConfig = new CANcoderConfiguration();
      absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      absolutedEncoderConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.absolutedEncoderOffset;
      armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);
  
      //Motor Configurations
      intakewheel.getConfigurator().apply(wheelConfig);
      intakewheel.getConfigurator().apply(speedMotionMagicConfigs);
      intakewheel.getConfigurator().apply(speedSlot0Configs);
      intakeArm.getConfigurator().apply(armConfig);
  
      // PID Controller and Feedforward
      armPID = new PIDController(EndEffectorConstants.armPID_Kp, EndEffectorConstants.armPID_Ki, EndEffectorConstants.armPID_Kd);
      armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      // armPID.setIntegratorRange(EndEffectorConstants.armPIDMinOutput, EndEffectorConstants.armPIDMaxOutput);
    }
  
    public void intakeCoral_Arm() {
      arriveAngle = EndEffectorConstants.coralStationAngle;
    }
  
    public void intakeCoral_Wheel() {
      intakewheel.setControl(request_EndEffectorSpeed.withVelocity(EndEffectorConstants.coralInSpeed_RotionPerSecond));
    }
  
    public void outCoral_L1_Arm() {
      arriveAngle = EndEffectorConstants.coralL1Angle;
    }
  
    public void outCoral_L1_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.coralL1OutVol);
    }
  
    public void outCoral_L2_Arm() {
      arriveAngle = EndEffectorConstants.coralL2Angle;
    }
  
    public void outCoral_L2_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.coralL2OutVol);
    }

    public void turnMore_Coral(){
      intakewheel.setVoltage(EndEffectorConstants.coralTurnMore);
    }
  
    public void outCoral_L3_Arm() {
      arriveAngle = EndEffectorConstants.coralL3Angle;
    }
  
    public void outCoral_L3_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.coralL3OutVol);
    }
  
    public void outCoral_L4_Arm() {
      arriveAngle = EndEffectorConstants.coralL4Angle;
    }

    public void coralL4Primitive_Arm(){
      arriveAngle = EndEffectorConstants.coralL4UpAngle;
    }
  
    public void outCoral_L4_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.coralL4OutVol);
    }
  
    public void shootNet_Arm() {
      arriveAngle = EndEffectorConstants.algaeNetAngle;
    }

    public void netPrimitive_Arm(){
      arriveAngle = EndEffectorConstants.netUpAngle;
    }
  
    public void shootNet_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.algaeShootNetVol);
    }
  
    public void shootProcessor_Arm() {
      arriveAngle = EndEffectorConstants.algaeProccesorAngle;
    }
  
    public void shootProcessor_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.algaeShootProcessorVol);
    }
  
    public void intakeAlgae_Low_Arm() {
      arriveAngle = EndEffectorConstants.algaeLowInAngle;
    }
  
    public void intakeAlgae_Low_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.algaeLowInVol);
    }
  
    public void intakeAlgae_High_Arm() {
      arriveAngle = EndEffectorConstants.algaeHighInAngle;
    }
  
    public void intakeAlgae_High_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.algaeHighInVol);
    }
  
    public void intakeAlgae_Floor_Arm() {
      arriveAngle = EndEffectorConstants.algaeFloorAngle;
    }
  
    public void intakeAlgae_Floor_Wheel() {
      intakewheel.setVoltage(EndEffectorConstants.algaeFloorInVol);
    }

    public void outAlgae() {
      intakewheel.setVoltage(EndEffectorConstants.algaeOutVol);
    }
  
    public void primitiveArm() {
      arriveAngle = EndEffectorConstants.primitiveAngle;
    }
  
    public void holdAlgae(){
      intakewheel.setVoltage(EndEffectorConstants.algaeHoldVol);
    }
  
    public void stopWheel() {
      intakewheel.setControl(request_EndEffectorSpeed.withVelocity(0));
    }
  
    public double getPosition() {
      return intakeArm.getPosition().getValueAsDouble();
    }
  
    public double getAbsolutePosition() {
      return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble();
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
  
    public boolean hasCoral() {
      return !irSensor_Coral.get();
    }

    // public boolean hasCoral(){
    //   nowTime_Coral = timer_Coral.get();
    //   if (shouldStart_Coral && sensorHasCoral()) {
    //     timer_Coral.reset();
    //     timer_Coral.start();
    //     shouldStart_Coral = false;
    //   }
    //   if (nowTime_Coral >= 0.02 && sensorHasCoral()) {
    //     timer_Coral.stop();
    //     shouldStart_Coral = false;
    //     return true;
    //   }else{
    //     shouldStart_Coral = false;
    //     return false;
    //   }
    // }
  
    public boolean sensorHasAlgae() {
      return !irSensor_Algae.get();
    }

    public boolean hasAlgae(){
      nowTime_Algae = timer_Algae.get();
      if (shouldStart_Algae && sensorHasAlgae()) {
        timer_Algae.reset();
        timer_Algae.start();
        shouldStart_Algae = false;
      }
      if (nowTime_Algae >= 0.5 && sensorHasAlgae()) {
        timer_Algae.stop();
        shouldStart_Algae = true;
        return true;
      }else {
        shouldStart_Algae = false;
        return false;
      }
    }
  
    public boolean arriveSetPoint() {
      return (Math.abs(armPID.getError()) <= 1.5);
    }
  
    public void setNeedMotorTurn(boolean needTurn){
      shouldMotorTurn_HasItem = needTurn;
      shouldMotorReset = needTurn;
      intakewheel.getConfigurator().apply(turnMotionMagicConfigs);
      intakewheel.getConfigurator().apply(slot1Configs);
    }
  
    @Override
    public void periodic() {
      if (shouldMotorReset) {
        nowPosition = intakewheel.getPosition().getValueAsDouble();
        shouldMotorReset = false;
      }
      if (shouldMotorTurn_HasItem) {
        intakewheel.setControl(m_request.withPosition(nowPosition + 0.65));
        if (intakewheel.getPosition().getValueAsDouble() >= nowPosition + 0.65) {
          shouldMotorTurn_HasItem = false;
          intakewheel.getConfigurator().apply(speedMotionMagicConfigs);
          intakewheel.getConfigurator().apply(speedSlot0Configs);
        }
      }
      // Arm
      if(85 >= getAngle() && getAngle() > 80 || 75 >= getAngle() && getAngle() > 70) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      }else if(80 >= getAngle() && getAngle() > 75) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks4, EndEffectorConstants.armFeedforward_Kg4, EndEffectorConstants.armFeedforward_Kv4);
      }else if(70 >= getAngle() && getAngle() > 61.6){
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks2, EndEffectorConstants.armFeedforward_Kg2, EndEffectorConstants.armFeedforward_Kv2);
      }else {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks3, EndEffectorConstants.armFeedforward_Kg3, EndEffectorConstants.armFeedforward_Kv3);
      }
    pidOutput = armPID.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getRadians(), getVelocity())/12;
    pidOutput = Constants.setMaxOutput(pidOutput, EndEffectorConstants.armPIDMaxOutput);
  
    
    output = pidOutput + feedforwardOutput;
    intakeArm.set(output);

    //SmartDashboard
    SmartDashboard.putNumber("EndEffector/pidOutput", pidOutput);
    SmartDashboard.putNumber("EndEffector/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("EndEffector/Output", output);
    SmartDashboard.putBoolean("EndEffector/arriveSetpoint", arriveSetPoint());
    SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
    SmartDashboard.putBoolean("EndEffector/hasAlgae", hasAlgae());
    SmartDashboard.putBoolean("EndEffector/HasSensorAlgae", sensorHasAlgae());
    SmartDashboard.putNumber("EndEffector/AbsolutedArmPosition", getAbsolutePosition());
    SmartDashboard.putNumber("EndEffector/ArmAngle", getAngle());
    SmartDashboard.putNumber("EndEffector/getPosition", getPosition());
    SmartDashboard.putString("Mode", Mode.nowMode);
  }
}
