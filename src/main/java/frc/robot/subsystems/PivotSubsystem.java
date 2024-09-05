// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterCalibration;

public class PivotSubsystem extends SubsystemBase {
  private static DriveSubsystem m_driveSubsystem;
  private static final CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor,
      MotorType.kBrushless);
  private static final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  // private PID controller
  private final static double k_p = 0.025;
  private final static double k_i = 0;
  private final static double k_d = 0.0005;
  private final static double k_f = -0.05;

  private static final PIDController m_pid = new PIDController(k_p, k_i, k_d);
  //profile pid control? Work with gavin

  // finding position
  private final static double k_zeroPosition = 0.441;
  private final static double k_ticksToDegrees = 29.7 / 0.088;
  private final static double k_balanceAngle = 40;

  private InterpolatingDoubleTreeMap m_interpolation;
  private double m_setPoint = 0;

  private boolean m_manual = false;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
  }

  // description: stalls the motor (they don't move if motor shouldn't move)
  public void setBrakeMode(boolean brake) {
    m_pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  // description: sets the power
  public void setPower(double power) {
    m_manual = true;
    m_pivotMotor.set(power);
  }

  // description: sets the power to degrees
  // approach: use PID Controller to set the point
  public void setPositionDegrees(double angle) {
    m_manual = false;
    m_setPoint = angle;
    m_pid.setSetpoint(angle);
  }

  // description: returns angle necessary to shoot at speaker from current
  // position
  /*
   * approach: get the current position on the field,
   * use a for loop to find the two (rows - 1st column) that the current position
   * is between or on
   * use the two rows (2nd colum) and returns double for angle
   */
  // a 2D array that (1st column - position on field) (2nd colum - corresponding
  // angle)
  // Autos only, to be removed
  public double getPivotAngleFromDistanceFromSpeaker(double distance) {
  return Constants.getShooterCalib(Constants.k_front,m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters(),false);
  }

  // description: returns double of angle needed to shoot at speaker in future
  // position on the field
  /*
   * approach: by using the current speed (from drivesystem) & find the future
   * position (based on how long robot takes to set angle)
   * use a for loop to find the two rows (1st column) that the future position is
   * between or on
   * use the two rows (2nd colum) to used (math - linear) and return double for
   * angle
   */
  // a 2D array that (1st column - position on field) (2nd colum - corresponding
  // angle)
  public double getPivotAngleFromRobotPos(boolean predictFuture) {
    // double m_futurePos = m_driveSubsystem.getEstimatedFuturePos().getX();
    // double m_currentSpeed = m_driveSubsystem.get;

    // for (int i = 0; i < Constants.k_front.length; i++) {
    //   double currenDist = Constants.k_front[i].getDistance();
    //   for
    // }
  return 0;
  }

  // description: returns a double of current angle in degrees
  // aproach: subtract the current position w/ supposed starting position ->
  // convert difference into degrees
  public double getAngleInDegrees() {
    return (m_pivotEncoder.get() - k_zeroPosition) * k_ticksToDegrees;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Raw Ticks", m_pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Degrees", getAngleInDegrees());
    SmartDashboard.putNumber("Pivot ERROR", getAngleInDegrees() - m_setPoint);
    SmartDashboard.putNumber("Pivot set point", m_setPoint);

    // feet forward
    if (!m_manual) {
      // double feedForward = Math.sin(Math.toRadians(m_setPoint - k_balanceAngle)) * k_f;
      double currentAngle = getAngleInDegrees();
      double feedForward = Math.sin(Math.toRadians(currentAngle - k_balanceAngle)) * k_f;
      //might fix floor slamming problem
      feedForward += m_pid.calculate(currentAngle);

      SmartDashboard.putNumber("Pivot Power", feedForward);
      SmartDashboard.putNumber("PivotSetPoint", m_setPoint);
      // This method will be called once per scheduler run
      m_pivotMotor.set(feedForward);
    }
  }

}