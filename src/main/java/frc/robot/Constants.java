// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
  }

  public static Double[] optimate(double currentAngle, double goalAngle, double speedMetersPerSecond){
    Double[] goal = new Double[2];
    double delta = Math.abs(goalAngle - currentAngle);
    if (delta > (Math.PI / 2)) {
      goalAngle = goalAngle - Math.PI;
      speedMetersPerSecond = speedMetersPerSecond * -1;
    }
    goal[0] = goalAngle;
    goal[1] = speedMetersPerSecond;
    return goal;
  }

  public static class ModuleConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2RadPerSec = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;
    public static final double driveEncoderRot2RadPerMin = turningEncoderRot2RadPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final double drivePidController_Kp = 0;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.05;
    public static final double driveFeedforward_Kv = 2.58;

  }

  public class SwerveConstants {
    public static final int leftFrontDrive_ID = 4;
    public static final int leftBackDrive_ID = 3;
    public static final int rightFrontDrive_ID = 1;
    public static final int rightBackDrive_ID = 2;

    public static final int leftFrontTurning_ID = 8;
    public static final int leftBackTurning_ID = 7;
    public static final int rightFrontTurning_ID = 5;
    public static final int rightBackTurning_ID = 6;

    public static final int leftFrontAbsolutedEncoder_ID = 44;
    public static final int leftBackAbsolutedEncoder_ID = 43;
    public static final int rightFrontAbsolutedEncoder_ID = 41;
    public static final int rightBackAbsolutedEncoder_ID = 42;

    public static final double leftFrontOffset = -0.34326171;
    public static final double leftBackOffset = 0.22436523;
    public static final double rightFrontOffset = -0.34301757;
    public static final double rightBackOffset = 0.15209960;

    public static final int gyro_ID = 56;


    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;


    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2));

    
    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 720;
  }

  public class PhotonConstants {
    public static final double xPidController_Kp = 0;
    public static final double xPidController_Ki = 0;
    public static final double xPidController_Kd = 0;

    public static final double yPidController_Kp = 0;
    public static final double yPidController_Ki = 0;
    public static final double yPidController_Kd = 0;

    public static final double rotationPidController_Kp = 0;
    public static final double rotationPidController_Ki = 0;
    public static final double rotationPidController_Kd = 0;

    public static final double xPidMinOutput = -0.2;
    public static final double xPidMaxOutput = 0.2;
    public static final double yPidMinOutput = -0.2;
    public static final double yPidMaxOutput = 0.2;
    public static final double rotationPidMinOutput = -0.2;
    public static final double rotationPidMaxOutput = 0.2;

  }

  public class IntakeConstants {
    public static final int intakeWheel_ID = 0;
    public static final int intakeArm_ID = 0;

    public static final int elevator_FirstMotor_ID = 0;
    public static final int elevator_SecondMotor_ID = 0;

    public static final int armAbsolutedEncoder_ID = 0;

    public static final double absolutedEncoderOffset = 0;

    public static final double armPid_Kp = 0;
    public static final double armPid_Ki = 0;
    public static final double armPid_Kd = 0;

    public static final double armFeedforward_Ks = 0;
    public static final double armFeedforward_Kg = 0;
    public static final double armFeedforward_Kv = 0;

    public static final double floorAngle = 0;
    public static final double l1Angle = 0;
    public static final double l2l3Angle = 0;
    public static final double l4Angle = 0;
    public static final double coralStationAngle = 0;
    public static final double netAngle = 0;
    public static final double l2Angle_Algae = 0;
    public static final double l3Angle_Algae = 0;
    public static final double proccesorAngle = 0;

    public static final double floor_Vol = 0;
    public static final double l1_Vol = 0;
    public static final double l2l_Vol = 0;
    public static final double l4_Vol = 0;
    public static final double coralStation_Vol = 0;
    public static final double net_Vol = 0;
    public static final double l2Algae_Vol = 0;
    public static final double l3Algae_Vol = 0;
    public static final double proccesor_Vol = 0;

    public static final double feedForwardMax = 0;
    

  }
}
