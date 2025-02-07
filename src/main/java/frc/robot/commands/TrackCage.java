// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackCage extends Command {
  /** Creates a new TrackCage. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xPidMeasurements;
  private double yPidMeasurements;
  private double rotationPidMeasurements;

  private double xPidError;
  private double yPidError;
  private double rotationPidError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  private int backTarget_ID;

  public TrackCage(SwerveSubsystem swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Cage, PhotonConstants.xPidMaxOutput_Cage);
    yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Cage, PhotonConstants.yPidMaxOutput_Cage);
    rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Cage, PhotonConstants.rotationPidMaxOutput_Cage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    backTarget_ID = m_PhotonVisionSubsystem.getBackTargetID();

    if(m_PhotonVisionSubsystem.hasFrontTarget()) {
      if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
        // Y-PID calculations
        yPidMeasurements = m_PhotonVisionSubsystem.getYPidMeasurements_FrontRight();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_FrontRight);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_FrontRight;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_FrontRight);
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_FrontRight);
        rotationPidMeasurements = (rotationPidError > 3) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_FrontRight;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_FrontRight);
      // X-PID calculations
        xPidMeasurements = m_PhotonVisionSubsystem.getXPidMeasurements_FrontRight();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_FrontRight);
        if((yPidError) < 3 && (rotationPidError) < 0.05){
          xPidMeasurements = (xPidError) > 0.05 ? xPidMeasurements : 0;
          xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_FrontRight);
        } else {
          xPidOutput = 0;
        }
    }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYPidMeasurements_FrontLeft();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_FrontLeft);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_FrontLeft;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_FrontLeft);
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_FrontLeft);
      rotationPidMeasurements = (rotationPidError > 3) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_FrontLeft;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_FrontLeft);
    // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXPidMeasurements_FrontLeft();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_FrontLeft);
      if((yPidError) < 3 && (rotationPidError) < 0.05){
        xPidMeasurements = (xPidError) > 0.05 ? xPidMeasurements : 0;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_FrontLeft);
      } else {
        xPidOutput = 0;
      }
    }
  }else if(m_PhotonVisionSubsystem.hasBackTarget()) {
    if(backTarget_ID == 20 || backTarget_ID == 11) {
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYPidMeasurements_FrontLeft();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11);
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11);
      rotationPidMeasurements = (rotationPidError > 3) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11);
    // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXPidMeasurements_FrontLeft();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11);
      if((yPidError) < 3 && (rotationPidError) < 0.05){
        xPidMeasurements = (xPidError) > 0.05 ? xPidMeasurements : 0;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11);
      } else {
        xPidOutput = 0;
      }
    }else if(backTarget_ID == 21 || backTarget_ID == 10) {
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYPidMeasurements_FrontLeft();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10);
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10);
      rotationPidMeasurements = (rotationPidError > 3) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10);
    // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXPidMeasurements_FrontLeft();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10);
      if((yPidError) < 3 && (rotationPidError) < 0.05){
        xPidMeasurements = (xPidError) > 0.05 ? xPidMeasurements : 0;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10);
      } else {
        xPidOutput = 0;
      }
    }
  }
    // impl
    m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
