// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L1_Auto extends Command {
  /** Creates a new Coral_L1_Auto. */
 private final ElevatorSubsystem m_ElevatorSubsystem;
 private final EndEffectorSubsystem m_EndEffectorSubsystem;
  public Coral_L1_Auto(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;


    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.outCoral_L1();
    m_EndEffectorSubsystem.outCoral_L1_Arm();
    // m_EndEffectorSubsystem.primitiveArm();


    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(Math.abs(m_EndEffectorSubsystem.getAngle() - EndEffectorConstants.primitiveAngle) <= 1) {
    //   arriveEndEffectorPrimition = true;
    // }
    // if(arriveEndEffectorPrimition) {
    //   m_ElevatorSubsystem.outCoral_L1();
    //   if(Math.abs(m_ElevatorSubsystem.getCurrentPosition() - m_ElevatorSubsystem.getGoalPosition()) < 1) {
    //     m_EndEffectorSubsystem.outCoral_L1_Arm();
    //   }
    // }

    if(m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arriveSetPoint()) {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
