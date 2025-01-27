// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  private final PhotonCamera camera;

  private final PIDController xPidController;
  private final PIDController yPidController;
  private final PIDController zPidController;
  private final PIDController rotationPidController;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private Optional<MultiTargetPNPResult> results;
  private List<PhotonTrackedTarget> targets;
  private int target_ID;

  private double xPidOutput;
  private double yPidOutput;
  private double zPidoutput;
  private double rotationPidOutput;

  private double botXError;
  private double botYError;
  private double botZError;
  private double botRotationError;


  public PhotonVisionSubsystem() {
    camera = new PhotonCamera("Logetich");

    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    zPidController = new PIDController(PhotonConstants.zPidController_Kp, PhotonConstants.zPidController_Ki, PhotonConstants.zPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);

    xPidController.setIntegratorRange(-0.4, 0.4);
    yPidController.setIntegratorRange(-0.4, 0.4);
    rotationPidController.setIntegratorRange(-0.4, 0.4);

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

  public double getXPidOutput() {
    return xPidOutput;
  }

  public double getYPidOutput() {
    return yPidOutput;
  }

  public double getRotationPidOutput() {
    return rotationPidOutput;
  }

  public double getXPidError() {
    return xPidController.getError();
  }

  public double getYPidError() {
    return yPidController.getError();
  }

  public double getRotationError() {
    return rotationPidController.getError();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    target = result.getBestTarget();
    results = result.getMultiTagResult();
    targets = result.getTargets();
    if(hasTarget()) {

      botXError = getTargetPose().getX();
      botYError = getTargetPose().getY();
      botZError = getTargetPose().getZ();
      botRotationError = -Math.toDegrees(getTargetPose().getRotation().getAngle());

      xPidOutput = xPidController.calculate(botXError, 0);
      yPidOutput = yPidController.calculate(botYError, 0);
      zPidoutput = zPidController.calculate(botZError, 0);
      rotationPidOutput = rotationPidController.calculate(botRotationError, 0);

      target_ID = target.getFiducialId();

      SmartDashboard.putNumber("Photon/botXError", botXError);
      SmartDashboard.putNumber("photon/botYError", botYError);
      SmartDashboard.putNumber("Photon/botZError", botZError);
      SmartDashboard.putNumber("Photon/botRotationError", botRotationError);

    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      zPidoutput = 0;
      rotationPidOutput = 0;
    }
  }
}
