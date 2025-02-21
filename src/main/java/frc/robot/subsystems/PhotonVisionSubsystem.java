// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  private final PhotonCamera frontRightCamera;
  private final PhotonCamera frontLeftCamera;
  private final PhotonCamera backCamera;

  private final Transform3d frontRightToRobot;
  private final Transform3d frontLeftToRobot;
  private final Transform3d backToRobot;


  private PhotonPipelineResult frontRightResult;
  private PhotonPipelineResult frontLeftResult;
  private PhotonPipelineResult backResult;
  private PhotonTrackedTarget frontRightTarget;
  private PhotonTrackedTarget frontLeftTarget;
  private PhotonTrackedTarget backTarget;
  private Optional<MultiTargetPNPResult> results;
  private List<PhotonTrackedTarget> frontRightTargets;
  private List<PhotonTrackedTarget> frontLeftTargets;
  private List<PhotonTrackedTarget> backTargets;

  private int frontRightTarget_ID;
  private int frontLeftTarget_ID;
  private int backTarget_ID;


  private double botXMeasurements_FrontRight;
  private double botYMeasurements_FrontRight;
  private double botRotationMeasurements_FrontRight;
  private double botXMeasurements_FrontLeft;
  private double botYMeasurements_FrontLeft;
  private double botRotationMeasurements_FrontLeft;
  private double botXMeasurements_Back;
  private double botYMeasurements_Back;
  private double botRotationMeasurements_Back;


  public PhotonVisionSubsystem() {
    frontRightCamera = new PhotonCamera("OV9281_FrontRight");
    frontLeftCamera = new PhotonCamera("OV9281_FrontLeft");
    backCamera = new PhotonCamera("OV9281_Back");

    frontRightToRobot = new Transform3d(0, 0, 0, new Rotation3d(new Rotation2d(0)));
    frontLeftToRobot = new Transform3d(0, 0, 0, new Rotation3d(new Rotation2d(0)));
    backToRobot = new Transform3d(0, 0, 0, new Rotation3d(new Rotation2d(0)));

  }

  public int getFrontRightTargetID() {
    return frontRightTarget_ID;
  }

  public int getFrontLeftTargetID() {
    return frontLeftTarget_ID;
  }

  public int getBackTargetID() {
    return backTarget_ID;
  }

  public boolean hasFrontRightTarget() {
    return frontRightResult.hasTargets();
  }

  public boolean hasFrontLeftTarget() {
    return frontLeftResult.hasTargets();
  }

  public boolean hasBackTarget() {
    return backResult.hasTargets();
  }

  public boolean hasFrontTarget() {
    if(hasFrontRightTarget() || hasFrontLeftTarget()) return true;
    return false;
  }

  public boolean hasTarget() {
    if(hasFrontTarget() || hasBackTarget()) return true;
    return false;
  }

  public Transform3d getFrontRightTargetPose() {
    return frontRightTarget.getBestCameraToTarget();
  }

  public Transform3d getFrontLeftTargetPose() {
    return frontLeftTarget.getBestCameraToTarget();
  }

  public Transform3d getBackTargetPose() {
    return backTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_FrontRight() {
    return frontRightTarget.getBestCameraToTarget().plus(frontRightToRobot);
  }

  public Transform3d getRobotToTargetPose_FrontLeft() {
    return frontLeftTarget.getBestCameraToTarget().plus(frontLeftToRobot);
  }

  public Transform3d getRobotToTargetPose_Back() {
    return backTarget.getBestCameraToTarget().plus(backToRobot);
  }

  public Optional<Matrix<N3, N3>> getCameraMatrix(String camera) {
    if(camera == "FrontRight") return frontRightCamera.getCameraMatrix();
    if(camera == "FrontLeft") return frontLeftCamera.getCameraMatrix();
    if(camera == "Back") return backCamera.getCameraMatrix();
    return null;
  }

  public Optional<Matrix<N8, N1>> getCameraDistCoeffs(String camera) {
    if(camera == "FrontRight") return frontRightCamera.getDistCoeffs();
    if(camera == "FrontLeft") return frontLeftCamera.getDistCoeffs();
    if(camera == "Back") return backCamera.getDistCoeffs();
    return null;
  }

  public double getXPidMeasurements_FrontRight() {
    return botXMeasurements_FrontRight;
  }

  public double getYPidMeasurements_FrontRight() {
    return botYMeasurements_FrontRight;
  }

  public double getRotationMeasurements_FrontRight() {
    return botRotationMeasurements_FrontRight;
  }

  public double getXPidMeasurements_FrontLeft() {
    return botXMeasurements_FrontLeft;
  }

  public double getYPidMeasurements_FrontLeft() {
    return botYMeasurements_FrontLeft;
  }

  public double getRotationMeasurements_FrontLeft() {
    return botRotationMeasurements_FrontLeft;
  }

  public double getXPidMeasurements_Back() {
    return botXMeasurements_Back;
  }

  public double getYPidMeasurements_Back() {
    return botYMeasurements_Back;
  }

  public double getRotationMeasurements_Back() {
    return botRotationMeasurements_Back;
  }

  public PhotonPipelineResult getResult(String camera) {
    if(camera == "FrontRight") return frontRightResult;
    if(camera == "FrontLeft") return frontLeftResult;
    if(camera == "Back") return backResult;
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontRightResult = frontRightCamera.getLatestResult();
    frontRightTarget = frontRightResult.getBestTarget();
    // frontRightTargets = frontRightResult.getTargets();
    frontLeftResult = frontLeftCamera.getLatestResult();
    frontLeftTarget = frontLeftResult.getBestTarget();
    // frontLeftTargets = frontLeftResult.getTargets();
    backResult = backCamera.getLatestResult();
    backTarget = backResult.getBestTarget();
    // backTargets = backResult.getTargets();
    if(hasFrontRightTarget()) {
      botXMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getX();
      botYMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getY();
      botRotationMeasurements_FrontRight = Math.toDegrees(getRobotToTargetPose_FrontRight().getRotation().getAngle());

      frontRightTarget_ID = frontRightTarget.getFiducialId();
      

      SmartDashboard.putNumber("Photon/BotXMeasurements_FrontRight", botXMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/BotYMeasurements_FrontRight", botYMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/BotRotatioMeasurements_FrontRight", botRotationMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FrontRightTarget_ID", frontRightTarget_ID);

    }else {
      botXMeasurements_FrontRight = 0;
      botYMeasurements_FrontRight = 0;
      botRotationMeasurements_FrontRight = 0;
    }
    if(hasFrontLeftTarget()) {
      botXMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getX();
      botYMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getY();
      botRotationMeasurements_FrontLeft = Math.toDegrees(getRobotToTargetPose_FrontLeft().getRotation().getAngle());

      frontLeftTarget_ID = frontLeftTarget.getFiducialId();
      

      // SmartDashboard.putNumber("Photon/BotXError_Front", botXMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotYError_Front", botYMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotRotationError_Front", botRotationMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/FrontTarget_ID", frontRightTarget_ID);

    }else {
      botXMeasurements_FrontLeft = 0;
      botYMeasurements_FrontLeft = 0;
      botRotationMeasurements_FrontLeft = 0;
    }
    if(hasBackTarget()) {
      botXMeasurements_Back = getRobotToTargetPose_Back().getX();
      botYMeasurements_Back = getRobotToTargetPose_Back().getY();
      botRotationMeasurements_Back = Math.toDegrees(getRobotToTargetPose_Back().getRotation().getAngle());

      backTarget_ID = backTarget.getFiducialId();
      

      // SmartDashboard.putNumber("Photon/BotXError_Front", botXMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotYError_Front", botYMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotRotationError_Front", botRotationMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/FrontTarget_ID", frontRightTarget_ID);

    }else {
      botXMeasurements_Back = 0;
      botYMeasurements_Back = 0;
      botRotationMeasurements_Back = 0;
      backTarget_ID = 0;
    }
  }
}
