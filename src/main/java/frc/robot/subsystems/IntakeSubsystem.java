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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakewheel;
  private final TalonFX intakeArm;

  private final TalonFX elevator_FirstMotor;
  private final TalonFX elevator_SecondMotor;

  private final TalonFXConfiguration wheelConfig;
  // private final Slot0Configs wheelSlot0Configs;

  private final TalonFXConfiguration armConfig;
  private final Slot0Configs armSlot0Configs;
  private final MotionMagicConfigs armMotionMagicConfigs;

  private final TalonFXConfiguration elevatorConfig;
  private final Slot0Configs elevatorSlot0Config;
  private final MotionMagicConfigs elevatorMotionMagicConfig;


  // private final VelocityVoltage request_Wheel;

  private final MotionMagicVoltage request_Arm;
  private final MotionMagicVoltage request_Elevator;


  private final CANcoder armAbsolutedEncoder;

  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController armPid;

  private final ArmFeedforward armFeedforward;

  private final DigitalInput irSensor;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;

  private double arriveAngle;
  private double goalPosition;



  public IntakeSubsystem() {
    intakewheel = new TalonFX(IntakeConstants.intakeWheel_ID);
    intakeArm = new TalonFX(IntakeConstants.intakeArm_ID);

    elevator_FirstMotor = new TalonFX(IntakeConstants.elevator_FirstMotor_ID);
    elevator_SecondMotor = new TalonFX(IntakeConstants.elevator_SecondMotor_ID);

    armAbsolutedEncoder = new CANcoder(IntakeConstants.armAbsolutedEncoder_ID);

    elevator_SecondMotor.setControl(new Follower(IntakeConstants.elevator_FirstMotor_ID, false));

    irSensor = new DigitalInput(IntakeConstants.irSensor_ID);

    arriveAngle = IntakeConstants.startAngle;
    goalPosition = IntakeConstants.startPosition;


    // Motor Configurations

    wheelConfig = new TalonFXConfiguration();
    // wheelSlot0Configs = new Slot0Configs();

    armConfig = new TalonFXConfiguration();
    armSlot0Configs = new Slot0Configs();
    armMotionMagicConfigs = new MotionMagicConfigs();


    elevatorConfig = new TalonFXConfiguration();
    elevatorSlot0Config = new Slot0Configs();
    elevatorMotionMagicConfig = new MotionMagicConfigs();

    // request_Wheel = new VelocityVoltage(0).withSlot(0);
    request_Arm = new MotionMagicVoltage(arriveAngle);
    request_Elevator = new MotionMagicVoltage(0);

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

    elevatorSlot0Config.kS = 0;
    elevatorSlot0Config.kG = 0;
    elevatorSlot0Config.kV = 0;
    elevatorSlot0Config.kA = 0;
    elevatorSlot0Config.kP = 0;
    elevatorSlot0Config.kI = 0;
    elevatorSlot0Config.kD = 0;

    elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 0;
    elevatorMotionMagicConfig.MotionMagicAcceleration = 0;
    elevatorMotionMagicConfig.MotionMagicJerk = 0;


    elevatorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakewheel.getConfigurator().apply(wheelConfig);
    // intakewheel.getConfigurator().apply(wheelSlot0Configs);

    intakeArm.getConfigurator().apply(armConfig);
    intakeArm.getConfigurator().apply(armSlot0Configs);
    intakeArm.getConfigurator().apply(armMotionMagicConfigs);

    elevator_FirstMotor.getConfigurator().apply(elevatorConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorConfig);
    elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_FirstMotor.getConfigurator().apply(elevatorMotionMagicConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorMotionMagicConfig);


    // Absolute Encoder Configurations
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.absolutedEncoderOffset;

    armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID Controllers

    armPid = new PIDController(IntakeConstants.armPid_Kp, IntakeConstants.armPid_Ki, IntakeConstants.armPid_Kd);

    armFeedforward = new ArmFeedforward(IntakeConstants.armFeedforward_Ks, IntakeConstants.armFeedforward_Kg, IntakeConstants.armFeedforward_Kv);

  }

  // void

  public void intakeCoral() {
    intakewheel.set(IntakeConstants.coralStation_Vol);
    this.arriveAngle = IntakeConstants.coralStationAngle;
    this.goalPosition = IntakeConstants.coralStationPosition; 
  }

  public void outCoral_L1() {
    intakewheel.set(IntakeConstants.l1_Vol);
    this.arriveAngle = IntakeConstants.l1Angle;
    this.goalPosition = IntakeConstants.l1Position;
  }

  public void outCoral_L2L3() {
    intakewheel.set(IntakeConstants.l2l3_Vol);
    this.arriveAngle = IntakeConstants.l2l3Angle;
    this.goalPosition = IntakeConstants.l2l3Position;
  }

  public void outCoral_L4() {
    intakewheel.set(IntakeConstants.l4_Vol);
    this.arriveAngle = IntakeConstants.l4Angle;
    this.goalPosition = IntakeConstants.l4Position;
  }

  public void shootNet() {
    intakewheel.set(IntakeConstants.net_Vol);
    this.arriveAngle = IntakeConstants.netAngle;
    this.goalPosition = IntakeConstants.netPosition;
  }

  public void intakeAlgae_low() {
    intakewheel.set(IntakeConstants.l2Algae_Vol);
    this.arriveAngle = IntakeConstants.l2Angle_Algae;
    this.goalPosition = IntakeConstants.l2Position_Algae;
  }

  public void intakeAlgae_high() {
    intakewheel.set(IntakeConstants.l3Algae_Vol);
    this.arriveAngle = IntakeConstants.l3Angle_Algae;
    this.goalPosition = IntakeConstants.l3Position_Algae;
  }

  public void intakeAlgae_floor() {
    intakewheel.set(IntakeConstants.floor_Vol);
    this.arriveAngle = IntakeConstants.floorAngle;
    this.goalPosition = IntakeConstants.floorPosition;
  }

  public void stopElevater() {
    this.goalPosition = IntakeConstants.startPosition;
  }

  public void stopArm() {
    arriveAngle = IntakeConstants.startAngle;
  }

  public void holdCoral() {
    intakewheel.set(IntakeConstants.holdCoral_Vol);
    // intakewheel.setControl(request_Wheel.withVelocity(IntakeConstants.holdCoralVelocity));
  }

  public void holdAlgae() {
    intakewheel.set(IntakeConstants.holdAlgae_Vol);
    // intakewheel.setControl(request_Wheel.withVelocity(IntakeConstants.holdAlgaeVelocity));
  }

  // get Value

  public double getCurrentPosition() {
    return elevator_FirstMotor.getPosition().getValueAsDouble();
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

  public boolean ifArrivePosition() {
    return (Math.abs(goalPosition - getCurrentPosition()) <= 1);
  }

  public boolean hasAlgae() {
    return intakewheel.getMotorStallCurrent().getValueAsDouble() > IntakeConstants.hasAlgaeCurrent;
  }

  public boolean hasCoral() {
    return !irSensor.get();
  }

  @Override
  public void periodic() {
    // Arm
    pidOutput = armPid.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getAngle(), pidOutput);
    if(feedforwardOutput >= IntakeConstants.feedForwardMax) {
      feedforwardOutput = IntakeConstants.feedForwardMax;
    }
    output = pidOutput + feedforwardOutput;

    intakeArm.set(output);

    // elevator
    elevator_FirstMotor.setControl(request_Elevator.withPosition(goalPosition));

    //SmartDashboard

    SmartDashboard.putNumber("Intake/CurrentPosition", getCurrentPosition());
    SmartDashboard.putBoolean("Intake/ifArrivePosition", ifArrivePosition());

    SmartDashboard.putNumber("Intake/pidOutput", pidOutput);
    SmartDashboard.putNumber("Intake/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("Intake/Output", output);
  }
}
