// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public Constants() {
    File f = new File("home/lvuser/practice");
    SmartDashboard.putString("Robot Name", "A#");
    if (!f.exists()) {
      SmartDashboard.putString("Robot Name", "Bb");
      States.m_isCompetition = true;

      // Camera
      DriveConstants.k_cameraFrontX = 6;
      DriveConstants.k_cameraFrontY = 9.5;
      DriveConstants.k_cameraBackX = 8.5;
      DriveConstants.k_cameraBackY = 11.5;

      // Pivot
      PivotConstants.k_pivotZeroAngle = 157.7;
      PivotConstants.k_isInverted = true;
      PivotConstants.k_intakePositionDegrees = 132;
      PivotConstants.k_f = .015;
      PivotConstants.k_p = .032;
      PivotConstants.k_i = 0.00005;
      PivotConstants.k_d = .0005;
      PivotConstants.k_iZone = 10;
      PivotConstants.k_resetPositionDegrees = 15;
      PivotConstants.k_offset = .3;
      PivotConstants.k_ampPositionDegrees = 22;
      PivotConstants.k_sourceAngle = 17;

      // Drive
      DriveConstants.k_FLOffset = 4.9 - (Math.PI / 2);
      DriveConstants.k_FROffset = 3.316;
      DriveConstants.k_BLOffset = 5.247 + (Math.PI);
      DriveConstants.k_BROffset = 6.11 + (Math.PI / 2);
      DriveConstants.k_maxSpeedMetersPerSecond = 5.74;

      // Front
      // FrontConstants.k_f = 1.5 / FrontConstants.k_maxVelocityRPM;
      // FrontConstants.k_p = 0.0006; // 0.00005
      // FrontConstants.k_i = 0.000003; // 0.0000001
      // FrontConstants.k_d = 0.001;
      // FrontConstants.k_iZone = 100;

      FrontConstants.k_f = 1 / FrontConstants.k_maxVelocityRPM;
      FrontConstants.k_p = 0.0005; // 0.00005
      FrontConstants.k_i = 0.00000; // 0.0000001
      FrontConstants.k_d = 0;
      FrontConstants.k_iZone = 100;

      // Back
      BackConstants.k_f = 1.1 / FrontConstants.k_maxVelocityRPM;
      BackConstants.k_p = .00075;
      BackConstants.k_i = .0000001;
      BackConstants.k_d = 0;
      BackConstants.k_iZone = 200;
      BackConstants.k_intakeVelocityRPM = 500;

      // Interpolation Table
      PivotConstants.k_distancesFront = new double[] {
          1.3,
          2,
          2.55,
          3.1,
          3.5,
          3.75,
          4,
          4.25,
          4.5,
          4.75,
          5,
          5.25,
          5.5,
          5.8,
          6.25,
          7,
          7.25, 
          13
      };
      //THIS ONE BMR
      PivotConstants.k_anglesFront = new double[] {
          10, // 1.3,
          22.9, // 2,
          29, // 2.55,
          36.4, // 3.1,
          37.3, // 3.5,
          39.8, // 3.75,
          41, // 4
          42, // 4.25,
          43.6, // 4.5,
          44.1, // 4.75,
          44.7, // 5,
          44.8, // 5.25,
          44.8, // 5.5
          44.3, // 5.8
          44.4, // 6.25
          44.1, // 7
          30, // 7.25
          30 // 13
      };
      PivotConstants.k_distancesBack = new double[] {
          1.4,
          1.75,
          2,
          2.25,
          2.5,
          2.75,
          3,
          3.5,
          4,
          8

      };
      PivotConstants.k_anglesBack = new double[] {
          120, // 1.5
          115, // 1.75
          119, // 2
          106, // 2.25
          105, // 2.5
          100, // 2.75
          100, // 3
          101, // 3.5
          98, // 4
          73 // 8
      };

    } else {
      SmartDashboard.putString("Robot Name", "A#");
      States.m_isCompetition = false;
    }
  }

  public static boolean m_allianceRed = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int k_FRTurningMotor = 4;
    public static final int k_FLTurningMotor = 2;
    public static final int k_BRTurningMotor = 6;
    public static final int k_BLTurningMotor = 8;

    public static final int k_FRDriveMotor = 3;
    public static final int k_FLDriveMotor = 1;
    public static final int k_BRDriveMotor = 5;
    public static final int k_BLDriveMotor = 7;

    public static final double k_driveDeadband = 0.1;

    public static final double k_driveRadius = .475953574;

    public static double k_FLOffset = 6.25 - (Math.PI / 2); // 4.8
    public static double k_FROffset = 6.28; // 5.33
    public static double k_BLOffset = 0.03 + (Math.PI); // 0.16
    public static double k_BROffset = 6.27 + (Math.PI / 2); // 4.87

    public static final int k_drivingMotorPinionTeeth = 14;

    public static final double k_driveWidth = Units.inchesToMeters(23.5);
    public static final double k_driveLength = Units.inchesToMeters(23.5);
    public static final double k_wheelDiameterMeters = .0762;
    public static final double k_drivingMotorReduction = (45.0 * 22) / (k_drivingMotorPinionTeeth * 15);

    public static final double k_driveTicksToMetersVelocity = ((k_wheelDiameterMeters * Math.PI)
        / k_drivingMotorReduction) / 60.0;
    public static final double k_driveTicksToMetersPosition = (k_wheelDiameterMeters * Math.PI)
        / k_drivingMotorReduction;
    public static final double k_turnTicksToDegreesVelocity = (2 * Math.PI) / 60.0;
    public static final double k_turnTicksToRadiansPosition = (2 * Math.PI);

    public static final double k_turningEncoderPositionPIDMinInput = 0; // radians
    public static final double k_turningEncoderPositionPIDMaxInput = k_turnTicksToRadiansPosition; // radians

    public static final boolean k_turningEncoderInverted = true;

    public static final double k_freeSpeedRPM = 6784;
    public static final double k_drivingMotorFreeSpeedRps = k_freeSpeedRPM / 60.0;
    public static final double k_wheelCircumferenceMeters = k_wheelDiameterMeters * Math.PI;
    public static final double k_driveWheelFreeSpeedRps = (k_drivingMotorFreeSpeedRps * k_wheelCircumferenceMeters)
        / k_drivingMotorReduction;

    // Swerve Module Drive PID
    public static final double k_driveP = 0.04;
    public static final double k_driveI = 0;
    public static final double k_driveD = 0;
    public static final double k_driveFF = 1 / k_driveWheelFreeSpeedRps;
    public static final double k_drivingMinOutput = -1;
    public static final double k_drivingMaxOutput = 1;

    // Swerve Module Turn PID
    public static final double k_turnP = 1;
    public static final double k_turnI = 0;
    public static final double k_turnD = 0;
    public static final double k_turnFF = 0;
    public static final double k_turningMinOutput = -1;
    public static final double k_turningMaxOutput = 1;

    public static final int k_driveMotorCurrentLimit = 50; // amps
    public static final int k_turnMotorCurrentLimit = 20; // amps

    // Driving Constants
    public static double k_maxSpeedMetersPerSecond = 5.74;
    public static final double k_maxDriveAcceleration = 7.427;
    public static final double k_maxAngularSpeed = Math.PI * 1.5; // radians per second
    public static final double k_maxAngularAcceleration = Math.PI;

    public static final double k_directionSlewRate = 3; // radians per second
    public static final double k_magnitudeSlewRate = 3.25; // percent per second (1 = 100%)
    public static final double k_rotationalSlewRate = 3; // percent per second (1 = 100%)

    public static final SwerveModuleState[] k_defaultState = {
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
    };

    public static final double k_rotateF = 0;
    public static final double k_rotateP = .016;
    public static final double k_rotateI = .05;// .01;
    public static final double k_rotateD = .0008;
    public static final double k_rotateIZone = 20; // 70
    public static final double k_rotateDeadzone = 2;
    public static final double k_maxRotInput = .8;

    public static final double k_lookAheadTimeSeconds = .2;

    public static final Translation2d k_rotatePoint = new Translation2d(0, 0);

    public static double k_cameraFrontX = 8.375;
    public static double k_cameraFrontY = 9.5;
    public static double k_cameraBackX = 11.5;
    public static double k_cameraBackY = 6;

    public static double k_cameraLeftX = -11.75;// + 10;
    public static double k_cameraLeftY = 5.6;
    public static double k_cameraRightX = 11.75;// - 7;
    public static double k_cameraRightY = -5.6;
  }

  public static class PivotConstants {

    public static double k_sourceAngle = 20;

    public static double k_f = .015;
    public static double k_p = 0.017; // .017
    public static double k_i = 0.02; // .02
    public static double k_d = 0; // 0
    public static double k_iZone = 10; // 10

    public static boolean k_isInverted = false;
    public static final int k_pivotMotor = 9;
    public static double k_pivotZeroAngle = 100;
    public static final double k_pivotTicksToDegrees = 360;
    public static double k_offset = 0;

    // Positions
    public static double k_intakePositionDegrees = 112;
    public static double k_resetPositionDegrees = 45;
    public static double k_ampPositionDegrees = 13;

    public static double[] k_distancesFront = {
        1.6,
        2.3,
        2.7,
        3,
        3.3,
        3.5,
        3.8,
        4,
        4.5,
        5,
        5.5,
        6
    };

    public static double[] k_anglesFront = {
        0, // 1.6
        15, // 2
        16.3, // 2.3
        16.7, // 2.7
        22.7, // 3
        24.5, // 3.3
        25.6, // 3.5
        26.8, // 3.8
        26.8, // 4
        26.8, // 4.5
        27.1, // 5
        26.2, // 5.5
        28.2, // 6
    };

    public static double[] k_distancesBack = {};

    public static double[] k_anglesBack = {};
  }

  // Positive is intake. negative is shoot intake side
  public static class FrontConstants {
    public static final int k_frontMotor = 11;
    public static double k_intakeVelocityRPM = 3000;
    public static final double k_speakerShootVelocityRPM = -5500; // 5000
    public static double k_ampShootVelocityRPM = -2250; // -1950
    public static final double k_maxVelocityRPM = 6250; // 6000

    public static final double k_speakerFeedPower = 1;
    public static final double k_deadzone = 250;

    public static final double k_adjustGamePiecePower = -.2;

    public static final int k_frontSensor = 1;
    public static final int k_backSensor = 2;

    public static double k_f = 1.1 / k_maxVelocityRPM;
    public static double k_p = 0.0005;//.00025;
    public static double k_i = 0;//.00052;
    public static double k_d = 0;
    public static double k_iZone = 0;

    public static final double[] k_revDistances = { 0, 2.00001, 5.8, 6.1, 6.2001, 8, 9, 10, 13, 17 };
    public static final double[] k_revSpeeds = { -4000, -5000, -5000, -5000, -5000, -3500, -4500, -4500, -4500, -3900 }; // {-5000, -5500, -5750, -4000,
                                                                                      // -2000}6
    public static final double k_idleRevSpeed = -2000;
  }

  // Positive is intake. negative is shoot intake side
  public static class BackConstants {
    public static final int k_backMotor = 10;
    public static double k_intakeVelocityRPM = 1000;
    public static final double k_speakerShootVelocityRPM = 5000;
    public static final double k_speakerFeedPower = -1;
    public static final double k_ampFeedPower = -0.35; // -500

    public static final double k_deadzone = 25;
    public static final double k_adjustGamePiecePower = -.2;

    public static double k_f = 1.0 / 5350;
    public static double k_p = 0.00004; // 0.00004
    public static double k_i = 0.0015; // 0.0015
    public static double k_d = 0; // 0
    public static double k_iZone = 50; // 50
  }

  public static class ElevatorConstants {
    public static final int k_elevatorMotor = 12;
    public static final double k_ticksToInches = -25.0 / 113.559;

    // manual command
    public static final double k_driveDeadband = 0;
    public static final double k_minDistance = 0;
    public static final double k_maxDistance = 17;

    public static final double k_zeroPoint = -1.814;
    // PID (kinda) tuning
    // public static final double k_P = 0.002;
    public static final double k_f = 0.9;
  }

  public static class StickConstants {
    public static final int k_leftStickMotor = 13;
    public static final int k_rightStickMotor = 14;

    public static final double k_maxExtentRotations = 3.4;
  }

  public static final double k_idealShootingDistanceMeters = 3;

  public static class States {
    public static boolean m_isCompetition = false;
    public static boolean m_speakerMode = true;
    public static boolean m_shootIntakeSide = false;
    public static boolean m_hasGamePiece = false;
    public static boolean m_isGamePieceStowed = false;
    public static boolean m_faceSpeaker = false;
    public static boolean m_intaking = false;
    public static boolean m_autoRotateAim = true;
    public static boolean m_slowMode = false;
    public static DriverStation.Alliance m_alliance = DriverStation.Alliance.Blue;

    public static Pose2d m_sourcePos = new Pose2d(14.6, .7, Rotation2d.fromDegrees(111.5));
    public static Pose2d m_ampPos = new Pose2d(1.9, 7.65, Rotation2d.fromDegrees(-90));
  }
}
