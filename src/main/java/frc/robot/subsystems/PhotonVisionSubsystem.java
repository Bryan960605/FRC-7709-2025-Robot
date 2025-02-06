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

  private final PhotonCamera frontCamera;
  private final PhotonCamera backCamera;

  private PhotonPipelineResult frontResult;
  private PhotonPipelineResult backResult;
  private PhotonTrackedTarget frontTarget;
  private PhotonTrackedTarget backTarget;
  private Optional<MultiTargetPNPResult> results;
  private List<PhotonTrackedTarget> frontTargets;
  private List<PhotonTrackedTarget> backTargets;

  private int frontTarget_ID;
  private int backTarget_ID;


  private double botXMeasurements_Front;
  private double botYMeasurements_Front;
  private double botRotationMeasurements_Front;
  private double botXMeasurements_Back;
  private double botYMeasurements_Back;
  private double botRotationMeasurements_Back;


  public PhotonVisionSubsystem() {
    frontCamera = new PhotonCamera("OV9287_Front");
    backCamera = new PhotonCamera("OV9287_Back");

  }

  public int getFrontTargetID() {
    return frontTarget_ID;
  }

  public int getBackTargetID() {
    return backTarget_ID;
  }

  public boolean hasFrontTarget() {
    return frontResult.hasTargets();
  }

  public boolean hasBackTarget() {
    return backResult.hasTargets();
  }

  public boolean hasTarget() {
    if(frontResult.hasTargets() || backResult.hasTargets()) {
      return true;
    }else {
      return false;
    }
  }

  public Transform3d getFrontTargetPose() {
    return frontTarget.getBestCameraToTarget();
  }

  public Transform3d getBackTargetPose() {
    return frontTarget.getBestCameraToTarget();
  }

  public double getXPidMeasurements_Front() {
    return botXMeasurements_Front;
  }

  public double getYPidMeasurements_Front() {
    return botYMeasurements_Front;
  }

  public double getRotationMeasurements_Front() {
    return botRotationMeasurements_Front;
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
    frontResult = frontCamera.getLatestResult();
    frontTarget = frontResult.getBestTarget();
    frontTargets = frontResult.getTargets();
    backTargets = backResult.getTargets();
    if(hasTarget()) {
      botXMeasurements_Front = getFrontTargetPose().getX();
      botYMeasurements_Front = getFrontTargetPose().getY();
      botRotationMeasurements_Front = -Math.toDegrees(getFrontTargetPose().getRotation().getAngle());
      botXMeasurements_Back = getBackTargetPose().getX();
      botYMeasurements_Back = getBackTargetPose().getY();
      botRotationMeasurements_Back = -Math.toDegrees(getBackTargetPose().getRotation().getAngle());

      frontTarget_ID = frontTarget.getFiducialId();
      backTarget_ID = backTarget.getFiducialId();
      

      SmartDashboard.putNumber("Photon/BotXError_Front", botXMeasurements_Front);
      SmartDashboard.putNumber("Photon/BotYError_Front", botYMeasurements_Front);
      SmartDashboard.putNumber("Photon/BotRotationError_Front", botRotationMeasurements_Front);
      SmartDashboard.putNumber("Photon/FrontTarget_ID", frontTarget_ID);

    }else {
      botXMeasurements_Front = 0;
      botYMeasurements_Front = 0;
      botRotationMeasurements_Front = 0;
      botXMeasurements_Back = 0;
      botYMeasurements_Back = 0;
      botRotationMeasurements_Back = 0;
      backTarget_ID = 0;
    }
  }
}
