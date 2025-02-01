// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakewheel;
  private final TalonFX intakeArm;

  private final TalonFXConfiguration wheelConfig;
  // private final Slot0Configs wheelSlot0Configs;

  private final TalonFXConfiguration armConfig;
  private final Slot0Configs armSlot0Configs;
  private final MotionMagicConfigs armMotionMagicConfigs;

  // private final VelocityVoltage request_Wheel;

  private final MotionMagicVoltage request_Arm;

  private final CANcoder armAbsolutedEncoder;

  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController armPid;

  private final ArmFeedforward armFeedforward;

  private final DigitalInput irSensor;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;

  private double arriveAngle;



  public EndEffectorSubsystem() {
    intakewheel = new TalonFX(EndEffectorConstants.intakeWheel_ID);
    intakeArm = new TalonFX(EndEffectorConstants.intakeArm_ID);

    armAbsolutedEncoder = new CANcoder(EndEffectorConstants.armAbsolutedEncoder_ID);


    irSensor = new DigitalInput(EndEffectorConstants.irSensor_ID);

    arriveAngle = EndEffectorConstants.startAngle;

    // Motor Configurations

    wheelConfig = new TalonFXConfiguration();
    // wheelSlot0Configs = new Slot0Configs();

    armConfig = new TalonFXConfiguration();
    armSlot0Configs = new Slot0Configs();
    armMotionMagicConfigs = new MotionMagicConfigs();

    // request_Wheel = new VelocityVoltage(0).withSlot(0);
    request_Arm = new MotionMagicVoltage(arriveAngle);

    // wheelSlot0Configs.kS = 0;
    // wheelSlot0Configs.kV = 0;
    // wheelSlot0Configs.kP = 0;
    // wheelSlot0Configs.kI = 0;
    // wheelSlot0Configs.kD = 0;

    armSlot0Configs.kS = 0;
    armSlot0Configs.kG = 0;
    armSlot0Configs.kV = 0;
    armSlot0Configs.kA = 0;
    armSlot0Configs.kP = 0;
    armSlot0Configs.kI = 0;
    armSlot0Configs.kD = 0;

    wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakewheel.getConfigurator().apply(wheelConfig);
    // intakewheel.getConfigurator().apply(wheelSlot0Configs);

    intakeArm.getConfigurator().apply(armConfig);
    intakeArm.getConfigurator().apply(armSlot0Configs);
    intakeArm.getConfigurator().apply(armMotionMagicConfigs);


    // Absolute Encoder Configurations
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.absolutedEncoderOffset;

    armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID Controller and Feedforward

    armPid = new PIDController(EndEffectorConstants.armPid_Kp, EndEffectorConstants.armPid_Ki, EndEffectorConstants.armPid_Kd);

    armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);

    armPid.setIntegratorRange(EndEffectorConstants.armPidMinOutput, EndEffectorConstants.armPidMaxOutput);

  }

  // void

  public void intakeCoral() {
    intakewheel.set(EndEffectorConstants.coralStation_Vol);
    this.arriveAngle = EndEffectorConstants.coralStationAngle;
  }

  public void outCoral_L1() {
    intakewheel.set(EndEffectorConstants.l1_Vol);
    this.arriveAngle = EndEffectorConstants.l1Angle;
  }

  public void outCoral_L2() {
    intakewheel.set(EndEffectorConstants.l2_Vol);
    this.arriveAngle = EndEffectorConstants.l2Angle;
  }

  public void outCoral_L3() {
    intakewheel.set(EndEffectorConstants.l3_Vol);
    this.arriveAngle = EndEffectorConstants.l3Angle;
  }

  public void outCoral_L4() {
    intakewheel.set(EndEffectorConstants.l4_Vol);
    this.arriveAngle = EndEffectorConstants.l4Angle;
  }

  public void shootNet() {
    intakewheel.set(EndEffectorConstants.net_Vol);
    this.arriveAngle = EndEffectorConstants.netAngle;
  }

  public void shootProcessor() {
    intakewheel.set(EndEffectorConstants.proccesor_Vol);
    this.arriveAngle = EndEffectorConstants.proccesorAngle;
  }

  public void intakeAlgae_Low() {
    intakewheel.set(EndEffectorConstants.l2Algae_Vol);
    this.arriveAngle = EndEffectorConstants.l2Angle_Algae;
  }

  public void intakeAlgae_High() {
    intakewheel.set(EndEffectorConstants.l3Algae_Vol);
    this.arriveAngle = EndEffectorConstants.l3Angle_Algae;
  }

  public void intakeAlgae_Floor() {
    intakewheel.set(EndEffectorConstants.floor_Vol);
    this.arriveAngle = EndEffectorConstants.floorAngle;
  }

  public void stopArm() {
    arriveAngle = EndEffectorConstants.startAngle;
  }

  public void stopWheel() {
    intakewheel.set(0);
  }

  public void holdCoral() {
    intakewheel.set(EndEffectorConstants.holdCoral_Vol);
    // intakewheel.setControl(request_Wheel.withVelocity(EndEffectorConstants.holdCoralVelocity));
  }

  public void holdAlgae() {
    intakewheel.set(EndEffectorConstants.holdAlgae_Vol);
    // intakewheel.setControl(request_Wheel.withVelocity(EndEffectorConstants.holdAlgaeVelocity));
  }

  // get Value

  public double getAngle() {
    return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getRadians() {
    return Units.degreesToRadians(getAngle());
  }

  public double getVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(armAbsolutedEncoder.getVelocity().getValueAsDouble()*60);
  }

  public boolean hasAlgae() {
    return intakewheel.getMotorStallCurrent().getValueAsDouble() > EndEffectorConstants.hasAlgaeCurrent;
  }

  public boolean hasCoral() {
    return !irSensor.get();
  }

  @Override
  public void periodic() {
    // Arm
    pidOutput = armPid.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getAngle(), pidOutput);
    if(feedforwardOutput >= EndEffectorConstants.feedForwardMax) {
      feedforwardOutput = EndEffectorConstants.feedForwardMax;
    }
    output = pidOutput + feedforwardOutput;

    intakeArm.set(output);

    //SmartDashboard

    SmartDashboard.putNumber("Intake/pidOutput", pidOutput);
    SmartDashboard.putNumber("Intake/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("Intake/Output", output);
  }
}
