// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kOperatorControllerPort = 1;
  }

  // public static Double[] optimate(double currentAngle, double goalAngle, double speedMetersPerSecond){
  //   Double[] goal = new Double[2];
  //   double delta = Math.abs(goalAngle - currentAngle);
  //   if (delta > (Math.PI / 2)) {
  //     goalAngle = goalAngle - Math.PI;
  //     speedMetersPerSecond = speedMetersPerSecond * -1;
  //   }
  //   goal[0] = goalAngle;
  //   goal[1] = speedMetersPerSecond;
  //   return goal;
  // }

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

    public static final double xPidMinOutput_Reef = -0.2;
    public static final double xPidMaxOutput_Reef = 0.2;
    public static final double yPidMinOutput_Reef = -0.2;
    public static final double yPidMaxOutput_Reef = 0.2;
    public static final double rotationPidMinOutput_Reef = -0.2;
    public static final double rotationPidMaxOutput_Reef = 0.2;

    public static final double xPidMinOutput_CoralStation = -0.2;
    public static final double xPidMaxOutput_CoralStation = 0.2;
    public static final double yPidMinOutput_CoralStation = -0.2;
    public static final double yPidMaxOutput_CoralStation = 0.2;
    public static final double rotationPidMinOutput_CoralStation = -0.2;
    public static final double rotationPidMaxOutput_CoralStation = 0.2;

    public static final double xPidMinOutput_Cage = -0.2;
    public static final double xPidMaxOutput_Cage = 0.2;
    public static final double yPidMinOutput_Cage = -0.2;
    public static final double yPidMaxOutput_Cage = 0.2;
    public static final double rotationPidMinOutput_Cage = -0.2;
    public static final double rotationPidMaxOutput_Cage = 0.2;

    public static final double xPidMinOutput_Net = -0.2;
    public static final double xPidMaxOutput_Net = 0.2;
    public static final double yPidMinOutput_Net = -0.2;
    public static final double yPidMaxOutput_Net = 0.2;
    public static final double rotationPidMinOutput_Net = -0.2;
    public static final double rotationPidMaxOutput_Net = 0.2;

    public static final double xPidMinOutput_Processor = -0.2;
    public static final double xPidMaxOutput_Processor = 0.2;
    public static final double yPidMinOutput_Processor = -0.2;
    public static final double yPidMaxOutput_Processor = 0.2;
    public static final double rotationPidMinOutput_Processor = -0.2;
    public static final double rotationPidMaxOutput_Processor = 0.2;

    public static final double xPidSetPoint_RightReef = 0; 
    public static final double yPidSetPoint_RightReef = 0;
    public static final double rotationPidSetPoint_RightReef = 0;

    public static final double xPidSetPoint_LeftReef = 0;
    public static final double yPidSetPoint_LeftReef = 0;
    public static final double rotationPidSetPoint_LeftReef = 0;

    public static final double xPidSetPoint_MiddleReef = 0;
    public static final double yPidSetPoint_MiddleReef = 0;
    public static final double rotationPidSetPoint_MiddleReef = 0;

    public static final double xPidSetPoint_CoralStation = 0;
    public static final double yPidSetPoint_CoralStation = 0;
    public static final double rotationPidSetPoint_CoralStation = 0;

    public static final double xPidSetPoint_Cage = 0;
    public static final double yPidSetPoint_Cage = 0;
    public static final double rotationPidSetPoint_Cage = 0;

    public static final double xPidSetPoint_Processor = 0;
    public static final double yPidSetPoint_Processor = 0;
    public static final double rotationPidSetPoint_Processor = 0;

    public static final double xPidSetPoint_Net = 0;
    public static final double yPidSetPoint_Net = 0;
    public static final double rotationPidSetPoint_Net = 0;
  }

  public class ElevatorConstants {
    public static final int elevator_FirstMotor_ID = 0;
    public static final int elevator_SecondMotor_ID = 0;

    public static final double primitivePosition = 0;
    public static final double coralL1Position = 0;
    public static final double coralL2Position = 0;
    public static final double coralL3Position = 0;
    public static final double coralL4Position = 0;
    public static final double coralStationPosition = 0;

    public static final double algaeFloorPosition = 0;
    public static final double algaeNetPosition = 0;
    public static final double algaeL2Position = 0;
    public static final double algaeL3Position = 0;
    public static final double algaeProccesorPosition = 0;
  }

  // public static class Mode{
  //   public static BooleanSupplier changeModeFunc = () -> nowModeIsCoral();
  //   public static String nowMode = "coralMode";
  //   public static boolean nowModeIsCoral(){
  //     return nowMode == "coral";
  //   }
  // }

  public static class EndEffectorConstants {
    public static final int intakeWheel_ID = 0;
    public static final int intakeArm_ID = 0;
    public static final int armAbsolutedEncoder_ID = 0;
    public static final int irSensor_Coral_ID = 0;
    public static final int irSensor_Algae_ID = 0;

    public static final double absolutedEncoderOffset = 0;

    public static final double armPID_Kp = 0;
    public static final double armPID_Ki = 0;
    public static final double armPID_Kd = 0;
    public static final double armPIDMinOutput = 0;
    public static final double armPIDMaxOutput = 0;

    public static final double armFeedforward_Ks = 0;
    public static final double armFeedforward_Kg = 0;
    public static final double armFeedforward_Kv = 0;

    public static final double primitiveAngle = 0;
    public static final double coralL1Angle = 0;
    public static final double coralL2Angle = 0;
    public static final double coralL3Angle = 0;
    public static final double coralL4Angle = 0;
    public static final double coralStationAngle = 0;
    public static final double algaeFloorAngle = 0;
    public static final double algaeNetAngle = 0;
    public static final double algaeLowInAngle = 0;
    public static final double algaeHighInAngle = 0;
    public static final double algaeProccesorAngle = 0;

    public static final double coralL1OutVol = 0;
    public static final double coralL2OutVol = 0;
    public static final double coralL3OutVol = 0;
    public static final double coralL4OutVol = 0;
    public static final double coralInVol = 0;
    public static final double algaeFloorInVol = 0;
    public static final double algaeShootNetVol = 0;
    public static final double algaeLowInVol = 0;
    public static final double algaeHighInVol = 0;
    public static final double algaeShootProcessorVol = 0;
    public static final double algaeHoldVol = 0;   
  }

  public static class ClimberConstants {
    public static final int climbMotor_ID = 0;
    public static final int absolutedEncoder_ID = 0;

    public static final boolean MotorReverse = false;

    public static final double absolutedEncoderOffset = 0;

    public static final double climbPID_Kp = 0;
    public static final double climbPID_Ki = 0;
    public static final double climbPID_Kd = 0;

    public static final double climbPIDMinRange = 0;
    public static final double climbPIDMaxRange = 0;

    public static final double climbPIDMinOutput = 0;
    public static final double climbPIDMaxOutput = 0;

    public static final double climbOutAngle = 0;
    public static final double climbInAngle = 0;
  }

  public class LEDConstants {
    public static final int candle_ID = 0;

    public static final int ledNum = 0;

    public static boolean LEDFlag = false;
    public static boolean hasGamePiece = false;
  }
}
