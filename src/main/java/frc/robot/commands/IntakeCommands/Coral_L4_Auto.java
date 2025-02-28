// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L4_Auto extends Command {
  /** Creates a new Coral_L4_Auto. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  public Coral_L4_Auto(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;


    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ElevatorSubsystem.outCoral_L4();
    // m_EndEffectorSubsystem.outCoral_L4_Arm();
    m_EndEffectorSubsystem.coralL4Primitive_Arm();


    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_EndEffectorSubsystem.arriveSetPoint()) {
      m_ElevatorSubsystem.outCoral_L4();    
      if(m_ElevatorSubsystem.arriveSetPoint()) {
        m_EndEffectorSubsystem.outCoral_L4_Arm();
      }
    }
    if(m_ElevatorSubsystem.arriveSetPoint() && Math.abs(m_EndEffectorSubsystem.getAngle() - EndEffectorConstants.coralL4Angle) <= 1) {
      m_EndEffectorSubsystem.outCoral_L4_Wheel();

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
    // m_ElevatorSubsystem.toPrimitive();
    // m_EndEffectorSubsystem.primitiveArm();
    // m_EndEffectorSubsystem.stopWheel();
    m_EndEffectorSubsystem.coralL4Primitive_Arm();
    while(!m_ElevatorSubsystem.arrivePrimition()) {
    if(m_EndEffectorSubsystem.arriveSetPoint()) {
      m_ElevatorSubsystem.toPrimitive();
      if(m_ElevatorSubsystem.arriveSetPoint()) {
        m_EndEffectorSubsystem.primitiveArm();
      }
    }
    }

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
