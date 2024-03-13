// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class MaxSwerveModule {
  private final CANSparkMax m_drive;
  private final CANSparkMax m_turn;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SwerveEncoders m_encoders;
  private boolean m_useDefaultEncoders = true;

  private final SparkPIDController m_drivePID;
  private final SparkPIDController m_turnPID;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MaxSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drive = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turn = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drive.restoreFactoryDefaults();
    m_turn.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drive.getEncoder();
    m_turningEncoder = m_turn.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivePID = m_drive.getPIDController();
    m_turnPID = m_turn.getPIDController();
    m_encoders = new SwerveEncoders(m_turn);
    m_drivePID.setFeedbackDevice(m_drivingEncoder);
    m_turnPID.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Constants.DriveConstants.k_driveTicksToMetersPosition);
    m_drivingEncoder.setVelocityConversionFactor(Constants.DriveConstants.k_driveTicksToMetersVelocity);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(Constants.DriveConstants.k_turnTicksToRadiansPosition);
    m_turningEncoder.setVelocityConversionFactor(Constants.DriveConstants.k_turnTicksToDegreesVelocity);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(Constants.DriveConstants.k_turningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turnPID.setPositionPIDWrappingEnabled(true);
    m_turnPID.setPositionPIDWrappingMinInput(Constants.DriveConstants.k_turningEncoderPositionPIDMinInput);
    m_turnPID.setPositionPIDWrappingMaxInput(Constants.DriveConstants.k_turningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivePID.setP(Constants.DriveConstants.k_driveP);
    m_drivePID.setI(Constants.DriveConstants.k_driveI);
    m_drivePID.setD(Constants.DriveConstants.k_driveD);
    m_drivePID.setFF(Constants.DriveConstants.k_driveFF);
    m_drivePID.setOutputRange(Constants.DriveConstants.k_drivingMinOutput,
        Constants.DriveConstants.k_drivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turnPID.setP(Constants.DriveConstants.k_turnP);
    m_turnPID.setI(Constants.DriveConstants.k_turnI);
    m_turnPID.setD(Constants.DriveConstants.k_turnD);
    m_turnPID.setFF(Constants.DriveConstants.k_turnFF);
    m_turnPID.setOutputRange(Constants.DriveConstants.k_turningMinOutput,
        Constants.DriveConstants.k_turningMaxOutput);

    m_drive.setIdleMode(IdleMode.kBrake);
    m_turn.setIdleMode(IdleMode.kBrake);
    m_drive.setSmartCurrentLimit(Constants.DriveConstants.k_driveMotorCurrentLimit);
    m_turn.setSmartCurrentLimit(Constants.DriveConstants.k_turnMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drive.burnFlash();
    m_turn.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_useDefaultEncoders ? m_turningEncoder.getPosition() : m_encoders.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d((m_useDefaultEncoders ? m_turningEncoder.getPosition() : m_encoders.getPosition()) - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d((m_useDefaultEncoders ? m_turningEncoder.getPosition() : m_encoders.getPosition()) - m_chassisAngularOffset));
  }

  public double getMotorPosRadians() {
    return m_encoders.getPosition();
  }

    public double getMagEncoderPosRadians() {
    return m_turningEncoder.getPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_useDefaultEncoders ? m_turningEncoder.getPosition() : m_encoders.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void setDesiredStateWithSpeed(SwerveModuleState desiredState, double speed) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_useDefaultEncoders ? m_turningEncoder.getPosition() : m_encoders.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivePID.setReference(optimizedDesiredState.speedMetersPerSecond * speed, CANSparkMax.ControlType.kVelocity);
    m_turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  // Manually run the turn motor for debugging purposes
  public void setTurn() {
    m_turn.set(.5);
  }

  // Manually run the drive motors for debugging purposes
  public void setDrive() {
    m_drive.set(.5);
  }
}