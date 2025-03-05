
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L4_Auto_RightReef extends Command {
  /** Creates a new Coral_L3_Auto. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem_Kraken m_SwerveSubsystem;

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
  public Coral_L4_Auto_RightReef(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, SwerveSubsystem_Kraken swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem, m_SwerveSubsystem, m_PhotonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_EndEffectorSubsystem.coralL4Primitive_Arm();
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = true;
    LEDConstants.LEDFlag = true;
    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.arrivedSetpoint()) {
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

    if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_RightReef);
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_RightReef;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_RightReef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontLeft();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_RightReef);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_RightReef;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_RightReef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_RightReef);
      xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_RightReef;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_RightReef);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidSetPoint_RightReef);
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }
    if(xPidMeasurements == PhotonConstants.xPidSetPoint_RightReef 
    && yPidMeasurements == PhotonConstants.yPidSetPoint_RightReef 
    && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_RightReef) {
      LEDConstants.arrivePosition_Base = true;
      LEDConstants.LEDFlag = true;
    }

    if(ElevatorConstants.arriveLow == false) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Reef);
    }
    m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = false;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
