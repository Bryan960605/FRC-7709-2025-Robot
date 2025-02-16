// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve_KrakenConstants;;

public class SwerveSubsystem_Kraken extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule_Kraken leftFront;
  private final SwerveModule_Kraken leftBack;
  private final SwerveModule_Kraken rightFront;
  private final SwerveModule_Kraken rightBack;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  private final SwerveDriveOdometry odometry;

  private final Field2d field;

  private RobotConfig robotConfig;

  //vision

  private final Transform3d robotToFrontRightCamera;
  private final Transform3d robotToFrontLeftCamera;
  private final Transform3d robotToBackCamera;

  private final PhotonPoseEstimator frontRightCameraEstimator;
  private final PhotonPoseEstimator frontLeftCameraEstimator;
  private final PhotonPoseEstimator backCameraEstimator;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private double currentTime;

  private PhotonPipelineResult frontRightCameraResult;
  private PhotonPipelineResult frontLeftCameraResult;
  private PhotonPipelineResult backCameraResult;

  private Transform3d fieldToFrontRightCamera;
  private Transform3d fieldToFrontLeftCamera;
  private Transform3d fieldToBackCamera;

  private Pose2d bestEstimatedPose2d;
  private Pose2d prevRobotEstimatedPose;

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private Matrix<N3, N1> stdDevs;
  private double[] stdDevsArray = {0.05, 0.05, 0.05};

  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;

  
  /**
   * 
   */
  public SwerveSubsystem_Kraken(PhotonVisionSubsystem photonVisionSubsystem) {
    leftFront = new SwerveModule_Kraken(
      Swerve_KrakenConstants.leftFrontTurning_ID,
      Swerve_KrakenConstants.leftFrontDrive_ID,
      Swerve_KrakenConstants.leftFrontAbsolutedEncoder_ID,
      Swerve_KrakenConstants.leftFrontOffset
            );
    leftBack = new SwerveModule_Kraken(
      Swerve_KrakenConstants.leftBackTurning_ID,
      Swerve_KrakenConstants.leftBackDrive_ID,
      Swerve_KrakenConstants.leftBackAbsolutedEncoder_ID,
      Swerve_KrakenConstants.leftBackOffset);
    rightFront = new SwerveModule_Kraken(
      Swerve_KrakenConstants.rightFrontTurning_ID,
      Swerve_KrakenConstants.rightFrontDrive_ID,
      Swerve_KrakenConstants.rightFrontAbsolutedEncoder_ID,
      Swerve_KrakenConstants.rightFrontOffset);
    rightBack = new SwerveModule_Kraken(
      Swerve_KrakenConstants.rightBackTurning_ID,
      Swerve_KrakenConstants.rightBackDrive_ID,
      Swerve_KrakenConstants.rightBackAbsolutedEncoder_ID,
      Swerve_KrakenConstants.rightBackOffset);

    gyro = new Pigeon2(Swerve_KrakenConstants.gyro_ID);
    gyroConfig = new Pigeon2Configuration();

    gyroConfig.MountPose.MountPoseYaw = 0;
    gyroConfig.MountPose.MountPosePitch = 0;
    gyroConfig.MountPose.MountPoseRoll = 0;

    gyro.getConfigurator().apply(gyroConfig);

    field = new Field2d();

    odometry = new SwerveDriveOdometry(Swerve_KrakenConstants.swerveKinematics, getRotation(), getModulesPosition(), new Pose2d());

    m_PhotonVisionSubsystem = photonVisionSubsystem;
    resetGyro();
     // All other subsystem initialization
    // ...
    //vision
    robotToFrontRightCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(getRotation()));
    robotToFrontLeftCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(getRotation()));
    robotToBackCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(getRotation()));

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);


    
    stdDevs = new Matrix<>(Nat.N3(), Nat.N1(), stdDevsArray);

    frontRightCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontRightCamera);
    frontLeftCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontLeftCamera);
    backCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCamera);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Swerve_KrakenConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose(), stdDevs, stdDevs);

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
            new PIDConstants(Swerve_KrakenConstants.pathingMoving_Kp, Swerve_KrakenConstants.pathingMoving_Ki, Swerve_KrakenConstants.pathingMoving_Kd), // Rotation PID constants
            new PIDConstants(Swerve_KrakenConstants.pathingtheta_Kp, Swerve_KrakenConstants.pathingtheta_Ki, Swerve_KrakenConstants.pathingtheta_Kd) // Translation PID constants
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

  public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevRobotEstimatedPose, PhotonPoseEstimator poseEstimator, Optional<Matrix<N3, N3>> cameraMatrix, Optional<Matrix<N8, N1>> cameraDistCoeffs, PhotonPipelineResult cameraResult) {
    // poseEstimator.setReferencePose(prevRobotEstimatedPose);
    return poseEstimator.update(cameraResult, cameraMatrix, cameraDistCoeffs);
  }

  public Pose2d chooseBestPose(Optional<EstimatedRobotPose> frontRightPose,Optional<EstimatedRobotPose> frontLeftPose, Optional<EstimatedRobotPose> backPose) {
    EstimatedRobotPose bestRobotPose;
    Pose2d bestRobotPose2d;
    if(frontRightPose.isPresent()) {
      bestRobotPose = frontRightPose.get();
      bestRobotPose2d = bestRobotPose.estimatedPose.toPose2d();
      return bestRobotPose2d;
    }
    if(frontLeftPose.isPresent()) {
      bestRobotPose = frontLeftPose.get();
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
    return Swerve_KrakenConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }


  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulesPosition() {
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      leftFront.getState(),
      rightFront.getState(),
      leftBack.getState(),
      rightBack.getState()
    };
  }

  public void setModouleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond);
      leftFront.setState(desiredStates[0]);
      rightFront.setState(desiredStates[1]);
      leftBack.setState(desiredStates[2]);
      rightBack.setState(desiredStates[3]);
  }

  public void setModouleStates_Auto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    rightFront.setState(desiredStates[1]);
    leftBack.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
}

  public void resetGyro() {
    gyro.reset();
  }

  public void setPose(Pose2d poses) {
    odometry.resetPosition(getRotation(), getModulesPosition(), poses);
  }

  public void setPose_Estimator(Pose2d poses) {
    swerveDrivePoseEstimator.resetPosition(getRotation(), getModulesPosition(), poses);
  }


  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
    SwerveModuleState[] state;
    xSpeed = xSpeed * Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond;
    ySpeed = ySpeed * Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond;
    zSpeed = zSpeed * Math.toRadians(Swerve_KrakenConstants.maxAngularVelocity_Angle);
    if(fieldOrient) {
      state = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));//之後要處理MaxSpeedPerSecond跟MaxRadianPerSecond的問題
    }else{
      state = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 

  public void autoDrive(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
    SwerveModuleState[] states = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);

    setModouleStates(states);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontRightCameraResult = m_PhotonVisionSubsystem.getResult("FrontRight");
    frontLeftCameraResult = m_PhotonVisionSubsystem.getResult("FrontLeft");
    backCameraResult = m_PhotonVisionSubsystem.getResult("Back");

    Optional<Matrix<N3, N3>> frontRightCameraMatrix = m_PhotonVisionSubsystem.getCameraMatrix("FrontRight");
    Optional<Matrix<N3, N3>> frontLeftCameraMatrix = m_PhotonVisionSubsystem.getCameraMatrix("FrontLeft");
    Optional<Matrix<N3, N3>> backCameraMatrix = m_PhotonVisionSubsystem.getCameraMatrix("Back");
    Optional<Matrix<N8, N1>> frontRightCameraDistCoeffs = m_PhotonVisionSubsystem.getCameraDistCoeffs("FrontRight");
    Optional<Matrix<N8, N1>> frontLeftCameraDistCoeffs = m_PhotonVisionSubsystem.getCameraDistCoeffs("FrontLeft");
    Optional<Matrix<N8, N1>> backCameraDistCoeffs = m_PhotonVisionSubsystem.getCameraDistCoeffs("Back");

    prevRobotEstimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    var frontRightRobotEstimatedPose = getEstimatedPose(prevRobotEstimatedPose, frontRightCameraEstimator, frontRightCameraMatrix, frontRightCameraDistCoeffs, frontRightCameraResult);
    var frontLeftRobotEstimatedPose = getEstimatedPose(prevRobotEstimatedPose, frontLeftCameraEstimator, frontLeftCameraMatrix, frontLeftCameraDistCoeffs, frontLeftCameraResult);
    var backRobotEstimatedPose = getEstimatedPose(prevRobotEstimatedPose, backCameraEstimator, backCameraMatrix, backCameraDistCoeffs, backCameraResult);
    bestEstimatedPose2d =  chooseBestPose(frontRightRobotEstimatedPose, frontLeftRobotEstimatedPose, backRobotEstimatedPose);

    odometry.update(getRotation(), getModulesPosition());
    swerveDrivePoseEstimator.update(getRotation(), getModulesPosition());
    // swerveDrivePoseEstimator.updateWithTime(currentTime, getRotation(), getModulesPosition());

    if(!(bestEstimatedPose2d == null)) {
      currentTime = MathSharedStore.getTimestamp();
      swerveDrivePoseEstimator.addVisionMeasurement(bestEstimatedPose2d, currentTime, stdDevs);
    }

    field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    field.setRobotPose(odometry.getPoseMeters());



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
}
