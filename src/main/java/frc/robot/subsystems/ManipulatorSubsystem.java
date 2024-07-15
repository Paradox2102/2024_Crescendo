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
  CANSparkFlex m_motor;
  private RelativeEncoder m_encoder;
  private boolean m_isFront;
  private final SparkPIDController m_PID;
  private double m_velocity = 0;
  private InterpolatingDoubleTreeMap m_revSpeeds = new InterpolatingDoubleTreeMap();

  /** Creates a new FrontSubsystem. */
  public ManipulatorSubsystem(DriveSubsystem driveSubsystem, Boolean isFront) {
    m_isFront = isFront;
    int id = (isFront ? Constants.ShooterConstants.k_frontMotor : Constants.HolderConstants.k_backMotor);
    SmartDashboard.putNumber("AmpVelo", 0);
    m_driveSubsystem = driveSubsystem;
    m_motor = new CANSparkFlex(id, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_PID = m_motor.getPIDController();
    m_motor.setSmartCurrentLimit(70);
    setBrakeMode(true);
    m_PID.setFF(m_isFront ? Constants.ShooterConstants.k_f : Constants.HolderConstants.k_f);
    m_PID.setP(m_isFront ? Constants.ShooterConstants.k_p : Constants.HolderConstants.k_p);
    m_PID.setI(m_isFront ? Constants.ShooterConstants.k_i : Constants.HolderConstants.k_i);
    m_PID.setIZone(m_isFront ? Constants.ShooterConstants.k_iZone : Constants.HolderConstants.k_iZone);
    m_PID.setD(m_isFront ? Constants.ShooterConstants.k_d : Constants.HolderConstants.k_d);

    setName(m_isFront ? "ShooterSubsystem" : "HolderSubsystem");
    m_motor.setInverted(m_isFront ? Constants.States.m_isCompetition : !Constants.States.m_isCompetition);
    m_motor.burnFlash();
  }

  // are the shooter rollers spinning at target RPM and ready to accurately shoot
  // the gamepiece
  // finished
  public boolean isReady() {
    return Math.abs(getVelocityRPM()) >= Math.abs(m_velocity)
        - (m_isFront ? Constants.ShooterConstants.k_deadzone : Constants.HolderConstants.k_deadzone);
  }

  // sets the power of the rollers
  // finished
  public void setPower(double power) {
    m_PID.setReference(power, ControlType.kDutyCycle);
  }

  // turns on and off the brake mode for the motors
  // finished
  public void setBrakeMode(boolean brake) {
    m_motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  // sets the RPM speed of the shooting motors
  // finished
  public void setVelocityRPM(double velocity) {
    m_velocity = velocity;
    m_PID.setReference(velocity, ControlType.kVelocity);
    SmartDashboard.putNumber(getName() + " Speed", Math.abs(m_velocity));
  }

  // reads the speed of the shooting rollers in RPM
  // finished
  public double getVelocityRPM() {
    return m_encoder.getVelocity();
  }

  // stops the motors
  // finished
  public void stop() {
    setPower(0);
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