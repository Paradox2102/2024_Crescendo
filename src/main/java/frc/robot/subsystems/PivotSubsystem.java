// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;
  CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor, MotorType.kBrushless);
  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);
  //PID
//private pid controller

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
  }

  //description: stalls the motor (they don't move if there is no power is used)
  //approach: if power is used than it is on coast mode else its on brake mode
  public void setBrakeMode(boolean brake) {
  }
  //description: sets the power
  //approach: m_pivotMotor(power);
  public void setPower(double power) {
    m_pivotMotor.set(power);
  }
  //description: sets the power to degrees
  //approach: use PID (new f depending on angle) to 
  public void setPositionDegrees(double angle) {
  }
  //description: returns angle necessary to shoot at speaker from current position
  /*approach: get the current position on the field, 
  *use a for loop to find the two (rows - 1st column) that the current position is between or on
  *use the two rows (2nd colum) and returns double for angle
  */
  //a 2D array that (1st column - position on field) (2nd colum - corresponding angle)
  // Autos only, to be removed
  public double getPivotAngleFromDistanceFromSpeaker(double distance) {
    return 0;
  }
  //description: returns double of angle needed to shoot at speaker in future position on the field
   /*approach: by using the current speed (from drivesystem) & find the future position (based on how long robot takes to set angle)
  *use a for loop to find the two rows (1st column) that the future position is between or on
  *use the two rows (2nd colum) to used (math - idk) and return double for angle
  */
  //a 2D array that (1st column - position on field) (2nd colum - corresponding angle)
  public double getPivotAngleFromRobotPos(boolean predictFuture) {
    return 0;
  }
  //description: returns a double of current angle in degrees
  //approach: get absoulute encoder multiply with math (sin?)
  public double getAngleInDegrees() {

    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
