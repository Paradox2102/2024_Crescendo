// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
  public static class ShooterCalibration {
    private final double distance;
    private final double angle;
    private final double speed;
    private static InterpolatingDoubleTreeMap k_shooterSpeed = new InterpolatingDoubleTreeMap();

    public static final double k_distanceOffsetInMeters = -.5;

    public ShooterCalibration(double distance, double angle, double speed) {
      this.distance = distance - k_distanceOffsetInMeters;
      this.angle = angle;
      this.speed = speed;
      // Use addRequirements() here to declare subsystem dependencies.
    }
    public double getDistance() {
      return distance;
    }
    public double getAngle(){
      return angle;
    }
    public double getSpeed(){
      return speed;
    }
  }

  public static ShooterCalibration[] k_front = new ShooterCalibration[] {
      new ShooterCalibration(1.36, 13, 4000),
      new ShooterCalibration(1.54, 15, 4000),
      new ShooterCalibration(1.78, 17, 4000),
      new ShooterCalibration(2.01, 19.5, 4000),
      new ShooterCalibration(2.44, 27, 4400),
      new ShooterCalibration(2.78, 29.5, 4400),
      new ShooterCalibration(3.08, 32, 4600),
      new ShooterCalibration(3.37, 34.5, 4700),
      new ShooterCalibration(3.48, 35, 4700),
      new ShooterCalibration(3.78, 36.5, 4800),
      new ShooterCalibration(4.05, 38, 4900),
      new ShooterCalibration(4.15, 39.7, 5000),
      new ShooterCalibration(4.32, 39.7, 5000),
      new ShooterCalibration(4.6, 40.5, 5000),
      new ShooterCalibration(4.95, 40.5, 5000),
      new ShooterCalibration(5.4, 41.2, 5200),
      new ShooterCalibration(5.8, 41.4, 5300),
      new ShooterCalibration(6.4, 42.0, 5400)
  };

  public static double getShooterCalib(ShooterCalibration[] data, double distance, boolean returnSpeed) {
    // if returnSpeed is true, returns the speed, otherwise returns the angle
    System.out.println("wahoiesfdsjfjdsjfjsdfjsdjfjsdjfsjfjdsjfsdjfjdsf");
    System.out.println(distance);
    for (int i = 1; i < data.length - 1; i++) {
      if ((distance >= data[i - 1].distance) && (distance < data[i].distance)) {
        double deltaD = distance - data[i - 1].distance;
        double D = data[i].distance - data[i - 1].distance;
        if (returnSpeed) {
          // speed
          double S = data[i].speed - data[i - 1].speed;
          double deltaS = (deltaD * (S / D));
          return data[i - 1].speed + deltaS;
        } else {
          // angle
          double A = data[i].angle - data[i - 1].angle;
          double deltaA = (deltaD * (A / D));
          return data[i - 1].angle + deltaA;
        }
      }
    }
    return (0);
  }

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
      PivotConstants.k_intakePositionDegrees = 130;
      PivotConstants.k_f = .015;
      PivotConstants.k_p = .032;
      PivotConstants.k_i = 0.00005;
      PivotConstants.k_d = .0005;
      PivotConstants.k_iZone = 10;
      PivotConstants.k_resetPositionDegrees = 10;
      PivotConstants.k_offset = .3;
      PivotConstants.k_ampPositionDegrees = 25;

      // Drive
      DriveConstants.k_FLOffset = 3.93 - (Math.PI / 2);
      DriveConstants.k_FROffset = 2.03;
      DriveConstants.k_BLOffset = 3.16 + (Math.PI);
      DriveConstants.k_BROffset = 1.01 + (Math.PI / 2);
      DriveConstants.k_maxSpeedMetersPerSecond = 4.8;

      // Shooter
      ShooterConstants.k_f = 1.1 / ShooterConstants.k_maxVelocityRPM;
      ShooterConstants.k_p = 0.0004; // 0.00005
      ShooterConstants.k_i = 0.0000003; // 0.0000001
      ShooterConstants.k_d = 0;
      ShooterConstants.k_iZone = 400;

      // Holder
      HolderConstants.k_f = 1.1 / ShooterConstants.k_maxVelocityRPM;
      HolderConstants.k_p = .00075;
      HolderConstants.k_i = .0000001;
      HolderConstants.k_d = 0;
      HolderConstants.k_iZone = 200;
      HolderConstants.k_intakeVelocityRPM = 500;

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
          7
      };
      PivotConstants.k_anglesFront = new double[] {
          10, // 1.3,
          22.9, // 2,
          29, // 2.55,
          36.9, // 3.1,
          37.5, // 3.5,
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
          44.1 // 7
      };

      ShooterCalibration[] k_back = new ShooterCalibration[] {
          new ShooterCalibration(1.4
          , 117, 0.0),
          new ShooterCalibration(1.75 , 112, 0),
          new ShooterCalibration(2 , 110, 0),
          new ShooterCalibration(2.25, 107, 0),
          new ShooterCalibration(2.5, 105, 0),
          new ShooterCalibration(2.75, 101, 0),
          new ShooterCalibration(3, 99, 0)
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
          117, // 1.5
          112, // 1.75
          110, // 2
          107, // 2.25
          105, // 2.5
          101, // 2.75
          99, // 3
          96, // 3.5
          95, // 4
          70 // 8
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

    public static double k_FLOffset = 2.77; // 4.8
    public static double k_FROffset = 0.01 + (Math.PI / 2); // 5.33
    public static double k_BLOffset = 4.2 - (Math.PI / 2); // 0.16
    public static double k_BROffset = 0.01 + (Math.PI); // 4.87

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
    public static double k_maxSpeedMetersPerSecond = 4.8;
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
    public static final double k_rotateP = .016;
    public static final double k_rotateI = .013;// .01;
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

    public static ShooterCalibration[] k_front;
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
    public static double k_resetPositionDegrees = 23;
    public static double k_ampPositionDegrees = 23;

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
        6.1
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
        28.2 // 6.1
    };

    // public static ShooterCalibration[] k_front = {
    // new ShooterCalibration(1.6, 0.0,0.0),
    // new ShooterCalibration(2.3, 15, 0),
    // new ShooterCalibration(2.7, 16.3, 0),
    // new ShooterCalibration(3, 16.7, 0),
    // new ShooterCalibration(3.3, 22.7, 0),
    // new ShooterCalibration(3.5, 24.5, 0),
    // new ShooterCalibration(3.8, 25.6, 0)
    // };

    public static double[] k_distancesBack = {};

    public static double[] k_anglesBack = {};
  }

  // Positive is intake. negative is shoot intake side
  public static class ShooterConstants {
    public static final int k_frontMotor = 11;
    public static double k_intakeVelocityRPM = 3000;
    public static double k_speakerShootVelocityRPM = -5000; // 5000
    public static double k_ampShootVelocityRPM = -1950; // -1950
    public static final double k_maxVelocityRPM = 6250; // 6000

    public static final double k_speakerFeedPower = 1;
    public static final double k_deadzone = 25;

    public static final double k_adjustGamePiecePower = -.2;

    public static final int k_frontSensor = 1;
    public static final int k_backSensor = 2;

    public static double k_f = 1.1 / k_maxVelocityRPM;
    public static double k_p = 0; // .00025
    public static double k_i = 0; // .00052
    public static double k_d = 0;
    public static double k_iZone = 0;

    public static final double[] k_revDistances = { 5.8, 6.5, 8, 10, 12 };
    public static final double[] k_revSpeeds = { -5000, -6000, -6000, -4000, -2000 }; // {-5000, -5500, -5750, -4000,
                                                                                      // -2000}
  }

  // Positive is intake. negative is shoot intake side
  public static class HolderConstants {
    public static final int k_backMotor = 10;
    public static double k_intakeVelocityRPM = 1000;
    public static final double k_speakerShootVelocityRPM = 5000; // 5000
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
    public static DriverStation.Alliance m_alliance = DriverStation.Alliance.Blue;
  }
}
