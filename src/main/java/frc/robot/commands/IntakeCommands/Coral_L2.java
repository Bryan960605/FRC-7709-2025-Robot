// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L2 extends Command {
  /** Creates a new Coral_L2. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private final BooleanSupplier ifFeedFunc;

  private boolean ifFeed;
  public Coral_L2(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    this.ifFeedFunc = ifFeed;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.outCoral_L2();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    if(m_ElevatorSubsystem.ifArrivePosition() && ifFeed) {
      m_EndEffectorSubsystem.outCoral_L2();
    }else {
      m_EndEffectorSubsystem.holdCoral();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stopElevater();
    m_EndEffectorSubsystem.stopArm();
    m_EndEffectorSubsystem.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
