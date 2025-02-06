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
public class TrackLeftReef extends Command {
  /** Creates a new TrackReef. */
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

  private int fiducialId;

  public TrackLeftReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Reef, PhotonConstants.xPidMaxOutput_Reef);
    yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Reef, PhotonConstants.yPidMaxOutput_Reef);
    rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Reef, PhotonConstants.rotationPidMaxOutput_Reef);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fiducialId = m_PhotonVisionSubsystem.getFrontTargetID();
    // Y-PID calculations
    yPidMeasurements = m_PhotonVisionSubsystem.getYPidMeasurements_Front();
    yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_LeftReef);
    yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_LeftReef;
    yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_LeftReef);
    // Rotation-PID calculations
    rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_Front();
    rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_LeftReef);
    rotationPidMeasurements = (rotationPidError > 3) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_LeftReef;
    rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_LeftReef);
    // X-PID calculations
    xPidMeasurements = m_PhotonVisionSubsystem.getXPidMeasurements_Front();
    xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_LeftReef);
    if((yPidError) < 3 && (rotationPidError) < 0.05){
      xPidMeasurements = (xPidError) > 0.05 ? xPidMeasurements : 0;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_LeftReef);
    }else {
      xPidOutput = 0;
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

  public PhotonVisionSubsystem getM_PhotonVisionSubsystem() {
    return m_PhotonVisionSubsystem;
  }

  public SwerveSubsystem getM_SwerveSubsystem() {
    return m_SwerveSubsystem;
  }

  public PIDController getRotationPidController() {
    return rotationPidController;
  }

  public void setRotationPidController(PIDController rotationPidController) {
    this.rotationPidController = rotationPidController;
  }

  public PIDController getxPidController() {
    return xPidController;
  }

  public void setxPidController(PIDController xPidController) {
    this.xPidController = xPidController;
  }

  public PIDController getyPidController() {
    return yPidController;
  }

  public void setyPidController(PIDController yPidController) {
    this.yPidController = yPidController;
  }

  public double getxPidMeasurements() {
    return xPidMeasurements;
  }

  public void setxPidMeasurements(double xPidMeasurements) {
    this.xPidMeasurements = xPidMeasurements;
  }

  public double getyPidMeasurements() {
    return yPidMeasurements;
  }

  public void setyPidMeasurements(double yPidMeasurements) {
    this.yPidMeasurements = yPidMeasurements;
  }

  public double getRotationPidMeasurements() {
    return rotationPidMeasurements;
  }

  public void setRotationPidMeasurements(double rotationPidMeasurements) {
    this.rotationPidMeasurements = rotationPidMeasurements;
  }

  public double getxPidError() {
    return xPidError;
  }

  public void setxPidError(double xPidError) {
    this.xPidError = xPidError;
  }

  public double getyPidError() {
    return yPidError;
  }

  public void setyPidError(double yPidError) {
    this.yPidError = yPidError;
  }

  public double getRotationPidError() {
    return rotationPidError;
  }

  public void setRotationPidError(double rotationPidError) {
    this.rotationPidError = rotationPidError;
  }

  public double getxPidOutput() {
    return xPidOutput;
  }

  public void setxPidOutput(double xPidOutput) {
    this.xPidOutput = xPidOutput;
  }

  public double getyPidOutput() {
    return yPidOutput;
  }

  public void setyPidOutput(double yPidOutput) {
    this.yPidOutput = yPidOutput;
  }

  public double getRotationPidOutput() {
    return rotationPidOutput;
  }

  public void setRotationPidOutput(double rotationPidOutput) {
    this.rotationPidOutput = rotationPidOutput;
  }

  public int getFiducialId() {
    return fiducialId;
  }

  public void setFiducialId(int fiducialId) {
    this.fiducialId = fiducialId;
  }
}
