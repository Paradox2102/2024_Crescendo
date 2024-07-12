// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;
  CANSparkFlex m_frontMotor;
  CANSparkFlex m_backMotor;
  private RelativeEncoder m_frontEncoder;
  private RelativeEncoder m_backEncoder;

  private final SparkPIDController m_frontPID;
  private final SparkPIDController m_backPID;
  private double m_shooterVelocity = 0;
  private double m_holderVelocity = 0;
  private InterpolatingDoubleTreeMap m_revSpeeds = new InterpolatingDoubleTreeMap();

  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem(DriveSubsystem driveSubsystem) {

    SmartDashboard.putNumber("AmpVelo", 0);

    m_backMotor = new CANSparkFlex(Constants.HolderConstants.k_backMotor, MotorType.kBrushless);
    m_backMotor.restoreFactoryDefaults();
    m_backEncoder = m_backMotor.getEncoder();
    m_backPID = m_backMotor.getPIDController();
    m_backMotor.setSmartCurrentLimit(70);
    
    m_backPID.setFF(Constants.HolderConstants.k_f);
    m_backPID.setP(Constants.HolderConstants.k_p);
    m_backPID.setI(Constants.HolderConstants.k_i);
    m_backPID.setIZone(Constants.HolderConstants.k_iZone);
    m_backPID.setD(Constants.HolderConstants.k_d);

    m_backMotor.setInverted(!Constants.States.m_isCompetition);

    m_driveSubsystem = driveSubsystem;
    m_frontMotor = new CANSparkFlex(Constants.ShooterConstants.k_frontMotor, MotorType.kBrushless);
    m_frontMotor.restoreFactoryDefaults();
    m_frontEncoder = m_frontMotor.getEncoder();
    m_frontPID = m_frontMotor.getPIDController();
    m_frontMotor.setSmartCurrentLimit(70);
    setBrakeMode(true);
    m_frontPID.setFF(Constants.ShooterConstants.k_f);
    m_frontPID.setP(Constants.ShooterConstants.k_p);
    m_frontPID.setI(Constants.ShooterConstants.k_i);
    m_frontPID.setIZone(Constants.ShooterConstants.k_iZone);
    m_frontPID.setD(Constants.ShooterConstants.k_d);
    m_frontMotor.setInverted(Constants.States.m_isCompetition);
    m_frontMotor.burnFlash();
  }

  // are the shooter rollers spinning at target RPM and ready to accurately shoot
  // the gamepiece
  public boolean isReady() {
    return Math.abs(getShooterVelocityRPM()) >= Math.abs(m_shooterVelocity)
        - Constants.ShooterConstants.k_deadzone
        && Math.abs(getHolderVelocityRPM()) >= Math.abs(m_holderVelocity)
            - Constants.HolderConstants.k_deadzone;
  }

  // sets the power of the rollers
  public void setShooterPower(double power) {
    if (Constants.States.m_shootIntakeSide) {
      m_frontPID.setReference(power, ControlType.kDutyCycle);
    } else {
      m_backPID.setReference(power, ControlType.kDutyCycle);
    }
  }

  public void setHolderPower(double power) {
    if (Constants.States.m_shootIntakeSide) {
      m_backPID.setReference(power, ControlType.kDutyCycle);
    } else {
      m_frontPID.setReference(power, ControlType.kDutyCycle);
    }
  }

  // turns on and off the brake mode for the motors
  // finished
  public void setBrakeMode(boolean brake) {
    m_frontMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_backMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  // sets the RPM speed of the shooting motors
  // TODO - fix automatic swapping of the front and back
  public void setShooterVelocityRPM(double velocity) {
    m_shooterVelocity = velocity;
    if (Constants.States.m_shootIntakeSide) {
      m_frontPID.setReference(velocity, ControlType.kVelocity);
    } else {
      m_backPID.setReference(velocity, ControlType.kVelocity);
    }

    SmartDashboard.putNumber(getName() + " Speed", Math.abs(m_shooterVelocity));
  }

  public void setHolderVelocityRPM(double velocity) {
    m_holderVelocity = velocity;
    if (Constants.States.m_shootIntakeSide) {
      m_backPID.setReference(velocity, ControlType.kVelocity);

    } else {
      m_frontPID.setReference(velocity, ControlType.kVelocity);
    }
    SmartDashboard.putNumber(getName() + " Speed", Math.abs(m_holderVelocity));
  }

  // reads the speed of the shooting rollers in RPM
  public double getShooterVelocityRPM() {
    if (Constants.States.m_shootIntakeSide) {
      return m_frontEncoder.getVelocity();
    } else {
      return m_backEncoder.getVelocity();
    }
  }

  public double getHolderVelocityRPM() {
    if (Constants.States.m_shootIntakeSide) {
      return m_backEncoder.getVelocity();
    } else {
      return m_frontEncoder.getVelocity();
    }
  }

  // stops the motors
  public void stopShooter() {
    setShooterPower(0);
  }

  public void stopHolder() {
    setHolderPower(0);
  }

  // returns the target RPM speed
  // finished
  public double getRevSpeed() {
    double distance = m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters();
    for (int i = 0; i < Constants.ShooterConstants.k_revDistances.length; i++) {
      m_revSpeeds.put(Constants.ShooterConstants.k_revDistances[i], Constants.ShooterConstants.k_revSpeeds[i]);
    }
    return m_revSpeeds.get(distance);
  }

  @Override
  public void periodic() {
  }
}
