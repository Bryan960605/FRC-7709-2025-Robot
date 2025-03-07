// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnSome extends Command {
  /** Creates a new turnSome. */
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private Timer timer;

  private boolean arrive;
  public TurnSome(EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    timer = new Timer();

    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    m_EndEffectorSubsystem.turnMore_Coral();
    arrive = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 0.5) {
      arrive = false;
    }else {
      arrive = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arrive;
  }
}
