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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX elevator_FirstMotor;
  private final TalonFX elevator_SecondMotor;

  private final TalonFXConfiguration elevatorConfig;
  private final Slot0Configs elevatorSlot0Config;
  private final MotionMagicConfigs elevatorMotionMagicConfig;

  private final MotionMagicVoltage request_Elevator;

  private double goalPosition;



  public ElevatorSubsystem() {
    elevator_FirstMotor = new TalonFX(ElevatorConstants.elevator_FirstMotor_ID);
    elevator_SecondMotor = new TalonFX(ElevatorConstants.elevator_SecondMotor_ID);

    elevator_SecondMotor.setControl(new Follower(ElevatorConstants.elevator_FirstMotor_ID, false));

    goalPosition = ElevatorConstants.startPosition;

    // Motor Configurations

    elevatorConfig = new TalonFXConfiguration();
    elevatorSlot0Config = new Slot0Configs();
    elevatorMotionMagicConfig = new MotionMagicConfigs();

    request_Elevator = new MotionMagicVoltage(0);

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
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevator_FirstMotor.getConfigurator().apply(elevatorConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorConfig);
    elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_FirstMotor.getConfigurator().apply(elevatorMotionMagicConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorMotionMagicConfig);

  }

  // void

  public void intakeCoral() {
    this.goalPosition = ElevatorConstants.coralStationPosition; 
  }

  public void outCoral_L1() {
    this.goalPosition = ElevatorConstants.l1Position;
  }

  public void outCoral_L2() {
    this.goalPosition = ElevatorConstants.l2Position;
  }

  public void outCoral_L3() {
    this.goalPosition = ElevatorConstants.l3Position;
  }

  public void outCoral_L4() {
    this.goalPosition = ElevatorConstants.l4Position;
  }

  public void shootNet() {
    this.goalPosition = ElevatorConstants.netPosition;
  }

  public void shootProcessor() {
    this.goalPosition = ElevatorConstants.proccesorPosition;
  }

  public void intakeAlgae_Low() {
    this.goalPosition = ElevatorConstants.l2Position_Algae;
  }

  public void intakeAlgae_High() {
    this.goalPosition = ElevatorConstants.l3Position_Algae;
  }

  public void intakeAlgae_Floor() {
    this.goalPosition = ElevatorConstants.floorPosition;
  }

  public void stopElevater() {
    this.goalPosition = ElevatorConstants.startPosition;
  }

  // get Value

  public double getCurrentPosition() {
    return elevator_FirstMotor.getPosition().getValueAsDouble();
  }

  public boolean ifArrivePosition() {
    return (Math.abs(goalPosition - getCurrentPosition()) <= 1);
  }

  @Override
  public void periodic() {
    // elevator
    elevator_FirstMotor.setControl(request_Elevator.withPosition(goalPosition));

    //SmartDashboard

    SmartDashboard.putNumber("Intake/CurrentPosition", getCurrentPosition());
    SmartDashboard.putBoolean("Intake/ifArrivePosition", ifArrivePosition());
  }
}
