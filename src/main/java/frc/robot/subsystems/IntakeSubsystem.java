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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
  private final TalonFXConfiguration armConfig;

  private final TalonFXConfiguration elevatorConfig;
  private final Slot0Configs elevatorSlot0Config;
  private final MotionMagicConfigs elevatorMotionMagicConfig;

  private final MotionMagicVoltage request;

  private final CANcoder armAbsolutedEncoder;

  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController armPid;

  private final ArmFeedforward armFeedforward;

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


    // Motor Configurations

    wheelConfig = new TalonFXConfiguration();
    armConfig = new TalonFXConfiguration();

    elevatorConfig = new TalonFXConfiguration();
    elevatorSlot0Config = new Slot0Configs();
    elevatorMotionMagicConfig = new MotionMagicConfigs();
    request = new MotionMagicVoltage(0);

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

    elevator_FirstMotor.getConfigurator().apply(elevatorConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorConfig);
    elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_FirstMotor.getConfigurator().apply(elevatorMotionMagicConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorMotionMagicConfig);

    intakewheel.getConfigurator().apply(wheelConfig);
    intakeArm.getConfigurator().apply(armConfig);


    // Absolute Encoder Configurations
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.absolutedEncoderOffset;

    armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID Controllers

    armPid = new PIDController(IntakeConstants.armPid_Kp, IntakeConstants.armPid_Ki, IntakeConstants.armPid_Kd);

    armFeedforward = new ArmFeedforward(IntakeConstants.armFeedforward_Ks, IntakeConstants.armFeedforward_Kg, IntakeConstants.armFeedforward_Kv);

  }

  public void intakeCoral() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.coralStationAngle;
  }

  public void outCoral_L1() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.l1Angle;
  }

  public void outCoral_L2L3() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.l2l3Angle;
  }

  public void outCoral_L4() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.l4Angle;
  }

  public void shootNet() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.netAngle;
  }

  public void intakeAlgae_low() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.l2Angle_Algae;
  }

  public void intakeAlgae_high() {
    intakewheel.set(0);
    this.arriveAngle = IntakeConstants.l3Angle_Algae;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutput = armPid.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getAngle(), pidOutput);
    if(feedforwardOutput >= IntakeConstants.feedForwardMax) {
      feedforwardOutput = IntakeConstants.feedForwardMax;
    }
    output = pidOutput + feedforwardOutput;

    // elevator
    elevator_FirstMotor.setControl(request.withPosition(goalPosition));

    SmartDashboard.putNumber("Intake/pidOutput", pidOutput);
    SmartDashboard.putNumber("Intake/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("Intake/Output", output);

    intakeArm.set(output);
  }
}
