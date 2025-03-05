// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L1_Test extends Command {
  /** Creates a new Coral_L1_Elevator. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;

  private final BooleanSupplier ifFeedFunc;

  private boolean ifFeed;
  private boolean hasArrivePosition;
  private boolean tooClose;
  public Coral_L1_Test(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, PhotonVisionSubsystem photonVisionSubsystem, BooleanSupplier ifFeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;

    this.ifFeedFunc = ifFeed;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ElevatorSubsystem.outCoral_L1();
    m_EndEffectorSubsystem.outCoral_L1_Arm();

    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
      hasArrivePosition = m_PhotonVisionSubsystem.getXMeasurements_FrontRight() < PhotonConstants.arriveXPosition_Reef ? true : false;
    }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      hasArrivePosition = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft() < PhotonConstants.arriveXPosition_Reef ? true : false;
    }else if(!m_PhotonVisionSubsystem.hasFrontTarget()) {
      hasArrivePosition = true;
    }

    if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
      tooClose = m_PhotonVisionSubsystem.getXMeasurements_FrontRight() < PhotonConstants.tooClosePosition_Reef ? true : false;
    }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      tooClose = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft() < PhotonConstants.tooClosePosition_Reef ? true : false;
    }else if(!m_PhotonVisionSubsystem.hasFrontTarget()) {
      tooClose = false;
    }

    if(hasArrivePosition) {
      if(tooClose == false) {
        m_ElevatorSubsystem.outCoral_L1();
      }else if(Math.abs(m_ElevatorSubsystem.getGoalPosition() - ElevatorConstants.coralL1Position) < 1) {
        m_ElevatorSubsystem.outCoral_L1();
        tooClose = false;
      }else {
        m_ElevatorSubsystem.toPrimitive();
      }
    }

    if(m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arrivedSetpoint() && ifFeed) {
      m_EndEffectorSubsystem.outCoral_L1_Wheel();

      LEDConstants.arrivePosition_Intake = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.toPrimitive();
    m_EndEffectorSubsystem.primitiveArm();
    m_EndEffectorSubsystem.stopWheel();

    LEDConstants.intakeArriving = false;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
