// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.TrackCage;
import frc.robot.commands.TrackCoralStation;
import frc.robot.commands.TrackNet;
import frc.robot.commands.TrackProcessor;
import frc.robot.commands.IntakeCommands.Coral_L1;
import frc.robot.commands.IntakeCommands.Coral_L2;
import frc.robot.commands.IntakeCommands.Coral_L3;
import frc.robot.commands.IntakeCommands.Coral_L4;
import frc.robot.commands.IntakeCommands.IntakeAlgae_High;
import frc.robot.commands.IntakeCommands.IntakeAlgae_Low;
import frc.robot.commands.IntakeCommands.ShootNet;
import frc.robot.commands.IntakeCommands.ShootProcessor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem = new PhotonVisionSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final EndEffectorSubsystem m_EffectorSubsystem = new EndEffectorSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
    DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
    DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);

    BooleanSupplier isSlowFunc = ()-> driverController.getHID().getLeftBumperButton();
    BooleanSupplier ifFeedFunc = ()-> operatorController.getHID().getAButton();

    driverController.b().whileTrue(
      Commands.runOnce(()->{
        m_SwerveSubsystem.resetGyro();
      })
    );

    driverController.rightBumper().whileTrue(new TrackCage(m_SwerveSubsystem, m_PhotonVisionSubsystem));
    driverController.a().whileTrue(new TrackCoralStation(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.x().whileTrue(new TrackCage(m_SwerveSubsystem, m_PhotonVisionSubsystem));
    driverController.y().whileTrue(new TrackProcessor(m_SwerveSubsystem, m_PhotonVisionSubsystem));
    driverController.pov(0).whileTrue(new TrackNet(m_SwerveSubsystem, m_PhotonVisionSubsystem));

    operatorController.pov(0).whileTrue(new Coral_L1(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));
    operatorController.b().whileTrue(new Coral_L2(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));
    operatorController.x().whileTrue(new Coral_L3(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));
    operatorController.y().whileTrue(new Coral_L4(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));

    operatorController.rightBumper().whileTrue(new IntakeAlgae_High(m_ElevatorSubsystem, m_EffectorSubsystem));
    operatorController.leftBumper().whileTrue(new IntakeAlgae_Low(m_ElevatorSubsystem, m_EffectorSubsystem));
    operatorController.rightTrigger().whileTrue(new ShootNet(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));
    operatorController.leftTrigger().whileTrue(new ShootProcessor(m_ElevatorSubsystem, m_EffectorSubsystem, ifFeedFunc));
    

    m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
