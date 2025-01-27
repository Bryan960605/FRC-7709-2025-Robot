// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackReef extends Command {
  /** Creates a new TrackReef. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  public TrackReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    xPidController.setIntegratorRange(-0.2, 0.2);
    yPidController.setIntegratorRange(-0.2, 0.2);
    rotationPidController.setIntegratorRange(-0.2, 0.2);
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PhotonVisionSubsystem.hasTarget()) {
      // Get target pose
      var targetPose = m_PhotonVisionSubsystem.getTargetPose();
      // Y-PID calculations
      double yMeasurements = m_PhotonVisionSubsystem.getYPidError();
      yMeasurements = Math.abs(yMeasurements) > 0.05 ? yMeasurements : 0;
      yPidOutput = -yPidController.calculate(yMeasurements, 0);
      // Rotation-PID calculations
      double rotationMeasurements = Units.radiansToDegrees(targetPose.getRotation().getAngle());
      rotationMeasurements = (Math.abs(rotationMeasurements)-180) > 3 ? rotationMeasurements : 0;
      rotationPidOutput = rotationPidController.calculate(rotationMeasurements, 180);
      // impl
      m_SwerveSubsystem.drive(0, yPidOutput, rotationPidOutput, false);
    }else{
      m_SwerveSubsystem.drive(0, 0, 0, false);
    }
    
    
    // if(m_PhotonVisionSubsystem.getRotationError() > 0.3) {
    //   rotationOutput = m_PhotonVisionSubsystem.getRotationPidOutput();
    // }else {
    //   rotationOutput = 0;
    // }
    // if(m_PhotonVisionSubsystem.getYPidError() > 0.3) {
    //   yPidOutput = m_PhotonVisionSubsystem.getYPidOutput();
    // }else {
    //   yPidOutput = 0;
    // }
  //   if(m_PhotonVisionSubsystem.getXPidError() > 0.3) {
  //     xPidOutput = m_PhotonVisionSubsystem.getXPidOutput();
  //   }else {
  //   xPidOutput = 0;
  // }
    // m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationOutput, false);

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
