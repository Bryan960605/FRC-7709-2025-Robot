// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule leftFront;
  private final SwerveModule leftBack;
  private final SwerveModule rightFront;
  private final SwerveModule rightBack;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  private final SwerveDriveOdometry odometry;

  private final Field2d field;

  private RobotConfig robotConfig;

  //vision
  private final PhotonCamera frontCamera;
  private final PhotonCamera backCamera;

  private final Transform3d robotToFrontCamera;
  private final Transform3d robotToBackCamera;

  private final PhotonPoseEstimator frontCameraEstimator;
  private final PhotonPoseEstimator backCameraEstimator;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private double currentTime;

  private PhotonPipelineResult frontCameraResult;
  private PhotonPipelineResult backCameraResult;

  private Pose2d bestPose2d;

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  
  /**
   * 
   */
  public SwerveSubsystem() {
    leftFront = new SwerveModule(
      SwerveConstants.leftFrontTurning_ID,
      SwerveConstants.leftFrontDrive_ID,
      SwerveConstants.leftFrontAbsolutedEncoder_ID,
      SwerveConstants.leftFrontOffset
            );
    leftBack = new SwerveModule(
      SwerveConstants.leftBackTurning_ID,
      SwerveConstants.leftBackDrive_ID,
      SwerveConstants.leftBackAbsolutedEncoder_ID,
      SwerveConstants.leftBackOffset);
    rightFront = new SwerveModule(
      SwerveConstants.rightFrontTurning_ID,
      SwerveConstants.rightFrontDrive_ID,
      SwerveConstants.rightFrontAbsolutedEncoder_ID,
      SwerveConstants.rightFrontOffset);
    rightBack = new SwerveModule(
      SwerveConstants.rightBackTurning_ID,
      SwerveConstants.rightBackDrive_ID,
      SwerveConstants.rightBackAbsolutedEncoder_ID,
      SwerveConstants.rightBackOffset);

    gyro = new Pigeon2(SwerveConstants.gyro_ID);
    gyroConfig = new Pigeon2Configuration();

    gyroConfig.MountPose.MountPoseYaw = 0;
    gyroConfig.MountPose.MountPosePitch = 0;
    gyroConfig.MountPose.MountPoseRoll = 0;

    gyro.getConfigurator().apply(gyroConfig);

    field = new Field2d();

    odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose());

    resetGyro();
     // All other subsystem initialization
    // ...
    //vision
    frontCamera = new PhotonCamera(getName());
    backCamera = new PhotonCamera(getName());

    robotToFrontCamera = new Transform3d(new Translation3d(null, null, null), new Rotation3d(getRotation()));
    robotToBackCamera = new Transform3d(new Translation3d(null, null, null), new Rotation3d(getRotation()));

    frontCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCamera);
    backCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCamera);

    aprilTagFieldLayout = aprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose(), null, null)


    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getRobotPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(SwerveConstants.pathingtheta_Kp, SwerveConstants.pathingtheta_Ki, SwerveConstants.pathingtheta_Kd), // Translation PID constants
                    new PIDConstants(SwerveConstants.pathingMoving_Kp, SwerveConstants.pathingMoving_Ki, SwerveConstants.pathingMoving_Kd) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    // // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentTime = Timer.getFPGATimestamp();
    var frontResults = frontCameraEstimator.update(frontCameraResult, null, null);
    var backResults = backCameraEstimator.update(backCameraResult, null, null);
    bestPose2d =  chooseBestPose(frontResults, backResults);

    odometry.update(getRotation(), getModulesPosition());
    // swerveDrivePoseEstimator.update(getRotation(), getModulesPosition());
    swerveDrivePoseEstimator.updateWithTime(currentTime, getRotation(), getModulesPosition());
    field.setRobotPose(odometry.getPoseMeters());

    if(!(bestPose2d == null)) {
      swerveDrivePoseEstimator.addVisionMeasurement(bestPose2d, currentTime, null);
    }



    SmartDashboard.putNumber("Swerve/leftFrontAbsolutePosition", leftFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/leftBackAbsolutePosition", leftBack.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightFrontAbsolutePosition", rightFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightBackAbsolutePosition", rightBack.getTurningPosition());

    SmartDashboard.putNumber("Swerve/leftFrontTurningMotorPosition", leftFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/leftBackTurningMotorPosition", leftBack.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightFrontTurningMotorPosition", rightFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightBackTurningMotorPosition", rightBack.getTurningMotorPosition());
    
    SmartDashboard.putNumber("Swerve/leftFrontDrivingMotorPosition", leftFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/leftBackDrivingMotorPosition", leftBack.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightFrontDrivingMotorPosition", rightFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightBackDrivingMotorPosition", rightBack.getDrivePosition());

    SmartDashboard.putNumber("Swerve/leftFrontVelocity", leftFront.getDriveVelocity());
  }


  public Pose2d chooseBestPose(Optional<EstimatedRobotPose> frontPose, Optional<EstimatedRobotPose> backPose) {
    EstimatedRobotPose bestRobotPose;
    Pose2d bestRobotPose2d;
    if(frontPose.isPresent()) {
      bestRobotPose = frontPose.get();
      bestRobotPose2d = bestRobotPose.estimatedPose.toPose2d();
      return bestRobotPose2d;
    }
    if(backPose.isPresent()) {
      bestRobotPose = backPose.get();
      bestRobotPose2d = bestRobotPose.estimatedPose.toPose2d();
      return bestRobotPose2d;
    }
    return null;
  }

  public ChassisSpeeds getChassisSpeed() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }


  public Pose2d getRobotPose() {
    return field.getRobotPose();
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulesPosition() {
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      leftBack.getPosition(),
      rightFront.getPosition(),
      rightBack.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      leftFront.getState(),
      leftBack.getState(),
      rightFront.getState(),
      rightBack.getState()
    };
  }

  public void setModouleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
      leftFront.setState(desiredStates[0]);
      leftBack.setState(desiredStates[1]);
      rightFront.setState(desiredStates[2]);
      rightBack.setState(desiredStates[3]);
  }

  public void setModouleStates_Auto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    leftBack.setState(desiredStates[1]);
    rightFront.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
}

  public void resetGyro() {
    gyro.reset();
  }

  public void setPose(Pose2d poses) {
    odometry.resetPosition(getRotation(), getModulesPosition(), poses);
  }


  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
    SwerveModuleState[] state;
    // xSpeed = xSpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    // ySpeed = ySpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    // zSpeed = zSpeed * Math.toRadians(SwerveConstants.maxAngularVelocity_Angle);
    if(fieldOrient) {
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));//之後要處理MaxSpeedPerSecond跟MaxRadianPerSecond的問題
    }else{
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 

  public void autoDrive(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
    SwerveModuleState[] states = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);

    setModouleStates(states);
  }
}
