// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

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

    public static double k_FLOffset = 1.67 - (Math.PI / 2);
    public static double k_FROffset = 2.21;
    public static double k_BLOffset = 2.25 + (Math.PI);
    public static double k_BROffset = 5.12 + (Math.PI / 2);

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

    public static final double k_freeSpeedRPM = 5676;
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
    public static final double k_maxSpeedMetersPerSecond = 4.8;
    public static final double k_maxDriveAcceleration = 3;
    public static final double k_maxAngularSpeed = Math.PI * 2; // radians per second
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

    public static final double k_rotateF = .2;
    public static final double k_rotateP = .01;
    public static final double k_rotateI = .015;
    public static final double k_rotateD = .001;
    public static final double k_rotateIZone = 70;
    public static final double k_rotateDeadzone = 2;

    public static final double k_lookAheadTimeSeconds = .2;
  }

  public static class PivotConstants {
    public static final int k_pivotMotor = 9;
    public static double k_pivotZeroAngle = 100;
    public static final double k_pivotTicksToDegrees = 360;

    // Positions
    public static final double k_intakePositionDegrees = 112;
    public static final double k_resetPositionDegrees = 0;
    public static final double k_ampPositionDegrees = -5;

    public static double[] k_distances = {
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
      6.1
    };

    public static double[] k_angles = {
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
      28.2 // 6.1
    };
  }

  // Positive is intake. negative is shoot intake side
  public static class ShooterConstants {
    public static final int k_shooterMotor = 11;
    public static final double k_intakeVelocityRPM = 3000;
    public static final double k_speakerShootVelocityRPM = -5000; // -5750
    public static final double k_ampShootVelocityRPM = -2000; // -1500
    
    public static final double k_speakerFeedVelocityRPM = 3500;
    public static final double k_deadzone = 25;

    public static final double k_adjustGamePiecePower = -1000;

    public static final int k_frontSensor = 2;
    public static final int k_backSensor = 1;
  }

  // Positive is intake. negative is shoot intake side
  public static class HolderConstants {
    public static final int k_holdingMotor = 10;
    public static final double k_intakeVelocityRPM = 1000;
    public static final double k_speakerShootVelocityRPM = 2000; 
    public static final double k_speakerFeedVelocityRPM = -9000;
    public static final double k_ampFeedVelocityRPM = -9000; // -500

    public static final double k_deadzone = 25;
    public static final double k_adjustGamePiecePower = -500;
  }

  public static class ElevatorConstants {
    public static final int k_elevatorMotor = 12;
    public static final double k_ticksToInches = -25.0/113.559;

//manual command
    public static final double k_driveDeadband = 0;
    public static final double k_minDistance = 0;
    public static final double k_maxDistance = 24;

    public static final double k_zeroPoint = -1.814;
//PID (kinda) tuning
    // public static final double k_P = 0.002;
    public static final double k_f = 0.9;
  }

  public static final double k_idealShootingDistanceMeters = 3;

  public static class States {
    public static boolean m_speakerMode = true;
    public static boolean m_shootIntakeSide = true;
    public static boolean m_hasGamePiece = false;
    public static boolean m_isGamePieceStowed = false;
    public static boolean m_runningShooterAndHolder = false;
    public static boolean m_faceSpeaker = false;
    public static boolean m_intaking = false;
  }

  public Constants() {
    File f = new File("home/lvuser/practice");
    if (true) {

      // Pivot
      PivotConstants.k_pivotZeroAngle = 0;

      // Drive
      DriveConstants.k_FLOffset = 1.81 - (Math.PI / 2);
      DriveConstants.k_FROffset = .04;
      DriveConstants.k_BLOffset = 4.19 + (Math.PI);
      DriveConstants.k_BROffset = 2.02 + (Math.PI / 2);
    }
  }
}
