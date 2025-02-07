// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  private final PhotonCamera frontRightCamera;
  private final PhotonCamera frontLeftCamera;
  private final PhotonCamera backCamera;

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
    frontRightCamera = new PhotonCamera("OV9287_FrontRight");
    frontLeftCamera = new PhotonCamera("OV9287_FrontLeft");
    backCamera = new PhotonCamera("OV9287_Back");

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
    backTargets = backResult.getTargets();
    if(hasTarget()) {
      botXMeasurements_FrontRight = getFrontRightTargetPose().getX();
      botYMeasurements_FrontRight = getFrontRightTargetPose().getY();
      botRotationMeasurements_FrontRight = -Math.toDegrees(getFrontRightTargetPose().getRotation().getAngle());
      botXMeasurements_FrontLeft = getFrontLeftTargetPose().getX();
      botYMeasurements_FrontLeft = getFrontLeftTargetPose().getY();
      botRotationMeasurements_FrontLeft = -Math.toDegrees(getFrontLeftTargetPose().getRotation().getAngle());
      botXMeasurements_Back = getBackTargetPose().getX();
      botYMeasurements_Back = getBackTargetPose().getY();
      botRotationMeasurements_Back = -Math.toDegrees(getBackTargetPose().getRotation().getAngle());

      frontRightTarget_ID = frontRightTarget.getFiducialId();
      frontLeftTarget_ID = frontLeftTarget.getFiducialId();
      backTarget_ID = backTarget.getFiducialId();
      

      SmartDashboard.putNumber("Photon/BotXError_Front", botXMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/BotYError_Front", botYMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/BotRotationError_Front", botRotationMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FrontTarget_ID", frontRightTarget_ID);

    }else {
      botXMeasurements_FrontRight = 0;
      botYMeasurements_FrontRight = 0;
      botRotationMeasurements_FrontRight = 0;
      botXMeasurements_FrontLeft = 0;
      botYMeasurements_FrontLeft = 0;
      botRotationMeasurements_FrontLeft = 0;
      botXMeasurements_Back = 0;
      botYMeasurements_Back = 0;
      botRotationMeasurements_Back = 0;
      backTarget_ID = 0;
    }
  }
}
