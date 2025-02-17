// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  private double m_power;
  // private double m_targetAngleInDegrees = 0; is this needed?

  private static final double k_deadzone = 0;
  private PIDController m_PID;
  private double m_setPoint = Constants.PivotConstants.k_resetPositionDegrees;

  //potentially use a structure with both angles and distances included for less complexity
  private double[] k_frontDistances = Constants.PivotConstants.k_distancesFront;
  private double[] k_backDistances = Constants.PivotConstants.k_distancesBack;

  private double[] k_anglesFront = Constants.PivotConstants.k_anglesFront;
  private double[] k_anglesBack = Constants.PivotConstants.k_anglesBack;

  private CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor, MotorType.kBrushless);
  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  private DoubleSupplier m_distanceFromSpeaker;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DoubleSupplier distance) {
    //SmartDashboard.putNumber("Amp Angle", 0);
    m_pivotMotor.restoreFactoryDefaults();
    m_PID = new PIDController(Constants.PivotConstants.k_p, Constants.PivotConstants.k_i, Constants.PivotConstants.k_d);
    //DoubleSupplier to return distance instead of passing in driveSubsystem
    m_distanceFromSpeaker = distance;
    //check highest current motor hits and make more reasonable limit
    m_pivotMotor.setSmartCurrentLimit(80);
    setBrakeMode(true);
    m_pivotEncoder.setPositionOffset(-0.8);
    m_PID.setIZone(Constants.PivotConstants.k_iZone);
    m_pivotMotor.setInverted(Constants.PivotConstants.k_isInverted);
    m_pivotMotor.burnFlash();
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  //does not work with PID
  public void setPower(double power) {
    m_power = power;
  }

  public void setPositionDegrees(double angle) {
    // Logger.log("PivotSubsystem", 1, String.format("setPositionDegrees=%f", angle));
    
    m_setPoint = angle;
  }

  // We should not be implementing code like this when we can just use InterpolatingDoubleTreeMap. - Gavin
  // Autos only, to be removed
  public double getPivotAngleFromDistanceFromSpeaker(double distance) {
    double[] distances = Constants.States.m_shootIntakeSide ? k_frontDistances : k_backDistances;
    double[] angles = Constants.States.m_shootIntakeSide ? k_anglesFront : k_anglesBack;
    if (distance > distances[distances.length - 1] || distance < distances[0]){
      return 0;
    }
    for (int i = 0; i < distances.length; i++) {
      if (distance > distances[i] && distance <= distances[i+1]) {
        double roc = (angles[i+1] - angles[i]) / (distances[i+1] - distances[i]);
        double dist = distance -distances[i];
        return (angles[i] + dist * roc) + PivotConstants.k_offset; 
      }
    }
    return 0;
  }

  public double getPivotAngleFromRobotPos(boolean predictFuture) {
    double[] distances = Constants.States.m_shootIntakeSide ? k_frontDistances : k_backDistances;
    double[] angles = Constants.States.m_shootIntakeSide ? k_anglesFront : k_anglesBack;
    double distance = m_distanceFromSpeaker.getAsDouble();
    if (distance > distances[distances.length - 1] || distance < distances[0]){
      return Constants.PivotConstants.k_resetPositionDegrees;
    }
    for (int i = 0; i < distances.length; i++) {
      if (distance > distances[i] && distance < distances[i+1]) {
        double roc = (angles[i+1] - angles[i]) / (distances[i+1] - distances[i]);
        double dist = distance - distances[i];
        return angles[i] + dist * roc; 
      }
    }
    return Constants.PivotConstants.k_resetPositionDegrees;
  }

  //for calibration
  private double getRawAngle() {
    return m_pivotEncoder.getAbsolutePosition();
  }

  public double getAngleInDegrees() {
    return ParadoxField.normalizeAngle(m_pivotEncoder.getAbsolutePosition() *  Constants.PivotConstants.k_pivotTicksToDegrees - Constants.PivotConstants.k_pivotZeroAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Pivot Raw Encoder Value", getRawAngle());
    SmartDashboard.putNumber("Pivot Angle in Degrees", getAngleInDegrees());
    double FF;
    double pid;
    double angle = getAngleInDegrees();

    //what is 40? create constant for that number
    FF = Constants.PivotConstants.k_f * Math.sin(Math.toRadians(angle - 40));
    if(Math.abs(getAngleInDegrees() - m_setPoint) > k_deadzone){
      pid = m_PID.calculate(angle, m_setPoint);
    } else {
      pid = 0;
    }
    m_power = FF + pid;
    // SmartDashboard.putNumber("Pivot Power", m_power);
    // SmartDashboard.putNumber("Pivot Calculated Error", Math.abs(getAngleInDegrees() - m_setPoint));
    SmartDashboard.putNumber("Pivot Set Point", m_setPoint);
    //Constants.PivotConstants.k_ampPositionDegrees = SmartDashboard.getEntry("Amp Angle").getDouble(0);
    m_pivotMotor.set(m_power);
  }
}
