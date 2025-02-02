// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  private final PhotonCamera camera1;
  private final PhotonCamera camera2;

  private final CameraTargetRelation cameraTargetRelation;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private Optional<MultiTargetPNPResult> results;
  private List<PhotonTrackedTarget> targets;
  private int target_ID;


  private double botXMeasurements;
  private double botYMeasurements;
  private double botRotationMeasurements;


  public PhotonVisionSubsystem() {
    camera1 = new PhotonCamera("OV9287_A");
    camera2 = new PhotonCamera("OV9287_B");

    cameraTargetRelation = new CameraTargetRelation(null, null);

  }

  public int getTargetID() {
    return target_ID;
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public Transform3d getTargetPose() {
    return target.getBestCameraToTarget();
  }

  public double getXPidMeasurements() {
    return botXMeasurements;
  }

  public double getYPidMeasurements() {
    return botYMeasurements;
  }

  public double getRotationMeasurements() {
    return botRotationMeasurements;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera1.getLatestResult();
    target = result.getBestTarget();
    results = result.getMultiTagResult();
    targets = result.getTargets();
    if(hasTarget()) {
      botXMeasurements = getTargetPose().getX();
      botYMeasurements = getTargetPose().getY();
      botRotationMeasurements = -Math.toDegrees(getTargetPose().getRotation().getAngle());

      target_ID = target.getFiducialId();

      SmartDashboard.putNumber("Photon/BotXError", botXMeasurements);
      SmartDashboard.putNumber("Photon/BotYError", botYMeasurements);
      SmartDashboard.putNumber("Photon/BotRotationError", botRotationMeasurements);
      SmartDashboard.putNumber("Photon/target_ID", target_ID);

    }else {
      botXMeasurements = 0;
      botYMeasurements = 0;
      botRotationMeasurements = 0;
      target_ID = 0;
    }
  }
}
