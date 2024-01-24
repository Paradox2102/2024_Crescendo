// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  // Create MaxSwerveModules
  private final MaxSwerveModule m_frontLeft = new MaxSwerveModule(
      Constants.DriveConstants.k_FLDriveMotor,
      Constants.DriveConstants.k_FLTurningMotor,
      Constants.DriveConstants.k_FLOffset);

  private final MaxSwerveModule m_frontRight = new MaxSwerveModule(
      Constants.DriveConstants.k_FRDriveMotor,
      Constants.DriveConstants.k_FRTurningMotor,
      Constants.DriveConstants.k_FROffset);

  private final MaxSwerveModule m_backLeft = new MaxSwerveModule(
      Constants.DriveConstants.k_BLDriveMotor,
      Constants.DriveConstants.k_BLTurningMotor,
      Constants.DriveConstants.k_BLOffset);

  private final MaxSwerveModule m_backRight = new MaxSwerveModule(
      Constants.DriveConstants.k_BRDriveMotor,
      Constants.DriveConstants.k_BRTurningMotor,
      Constants.DriveConstants.k_BROffset);
      

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.k_magnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.k_rotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final SwerveDriveKinematics m_swerve = new SwerveDriveKinematics(
        new Translation2d(.33655, .33655),
        new Translation2d(.33655, -.33655),
        new Translation2d(-.33655, .33655),
        new Translation2d(-.33655, -.33655));

    PositionTrackerPose m_tracker = null;
    ApriltagsCamera m_frontCamera;
    ApriltagsCamera m_backCamera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ApriltagsCamera frontCamera, ApriltagsCamera backCamera) {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);

    m_frontCamera = frontCamera;
    m_backCamera = backCamera;

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds, 
      this::driveWithChassisSpeedRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(1, 0 , 0), 
        new PIDConstants(1, 0, 0), 
        Constants.DriveConstants.k_maxSpeedMetersPerSecond, 
        .475953574, 
        new ReplanningConfig()), 
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this);

    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  // For debugging purposes (drive robot forward at full speed)
  public void setPower() {
    drive(1, 0, 0, true, false);
  }

  public SwerveDriveKinematics getSwerve() {
    return m_swerve;
  }

  public void setTracker(PositionTrackerPose tracker) {
    m_tracker = tracker;
  }

  public SwerveModulePosition[] getModulePosition() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Turn FR", (m_frontRight.getAngleRadians()));///Math.PI);
    SmartDashboard.putNumber("Turn FL", m_frontLeft.getAngleRadians());// - (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber("Turn BR", m_backRight.getAngleRadians());// + (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber("Turn BL", m_backLeft.getAngleRadians());// + (Math.PI)) / Math.PI);
    // SmartDashboard.putNumber("Pose Est X", (m_tracker.getPose2dFRC().getTranslation().getX()));
    // SmartDashboard.putNumber("Pose Est Y", (m_tracker.getPose2dFRC().getTranslation().getY()));
    // SmartDashboard.putNumber("Pose Est Rot", (m_tracker.getPose2dFRC().getRotation().getDegrees()));
    SmartDashboard.putNumber("PIgeon2", m_gyro.getAngle());

    // m_tracker.update(m_frontCamera, m_backCamera);
    // m_field.setRobotPose(m_tracker.getPose2dFRC().getTranslation().getX(), m_tracker.getPose2dFRC().getTranslation().getY(), m_tracker.getPose2dFRC().getRotation());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_tracker.getPose2dFRC();
  }

  public Field2d getField() {
    return m_field;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()};
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_swerve.toChassisSpeeds(getModuleStates());
  }

  public void driveWithChassisSpeedRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = m_swerve.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    // Pose2d pose = m_tracker.getPose2dFRC();
    // Logger.log("Robot Pose",0,String.format(",%f,%f,%f", pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_tracker.setXYAngleFRC(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.k_directionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.k_maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.k_maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.DriveConstants.k_maxAngularSpeed;

    var swerveModuleStates = m_swerve.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, ParadoxField.rotation2dFromFRC(m_tracker.getPose2d().getRotation()))
            : new ChassisSpeeds(-xSpeedDelivered, -ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Rotation2d getGyroRotation2d() {
    return m_gyro.getRotation2d();
  }

  public Pigeon2 getGyro() {
    return m_gyro;
  }

  public PositionTrackerPose getTracker() {
    return m_tracker;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStatesWithSpeed(SwerveModuleState[] desiredStates, double speed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredStateWithSpeed(desiredStates[0], speed);
    m_frontRight.setDesiredStateWithSpeed(desiredStates[1], speed);
    m_backLeft.setDesiredStateWithSpeed(desiredStates[2], speed);
    m_backRight.setDesiredStateWithSpeed(desiredStates[3], speed);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }
 
  // Debugging purposes (drives one motor)
  public void setOnePower() {
    m_backLeft.setDrive();
  }

  /** Zeroes the heading of the robot. */
  public void setHeading(double angle) {
    m_tracker.setXYAngleFRC(m_tracker.getPose2dFRC().getX(), m_tracker.getPose2dFRC().getY(), angle);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingInDegrees() {
    double angle = m_tracker.getPose2dFRC().getRotation().getDegrees();
    return ParadoxField.normalizeAngle(angle);
  }
}