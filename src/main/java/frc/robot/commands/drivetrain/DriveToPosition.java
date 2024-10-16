// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PositionTrackerPose;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPosition extends Command {
  /** Creates a new AutoTranslate. */
  DriveSubsystem m_subsystem;
  double m_xPos;
  double m_yPos;
  double m_rotationDegrees;
  PositionTrackerPose m_tracker;

  double m_currentX = 0;
  double m_currentY = 0;
  double m_currentRot = 0;

  private static final double k_f = .4;
  private static final double k_p = .003;
  private static final double k_i = 0;
  private static final double k_d = .0007;
  private static final double k_deadzoneMeters = .1;

  PIDController m_xPID = new PIDController(k_p, k_i, k_d);
  PIDController m_yPID = new PIDController(k_p, k_i, k_d);
  double m_rot = 0;

  public DriveToPosition(DriveSubsystem driveSubsystem, Pose2d pose) {
    m_subsystem = driveSubsystem;
    m_xPos = pose.getX();
    m_yPos = pose.getY();
    m_rotationDegrees = pose.getRotation().getDegrees();
    m_tracker = m_subsystem.getTracker();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_xPos);
    m_yPID.setSetpoint(m_yPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_subsystem.getTracker().getPose2d();
    m_currentX = currentPose.getX();
    m_currentY = currentPose.getY();
    m_currentRot = currentPose.getRotation().getDegrees();

    double xVelocity = m_xPID.calculate(m_currentX);
    double yVelocity = m_yPID.calculate(m_currentY);
    double rotVelocity = m_subsystem.orientPID(m_rotationDegrees);

    xVelocity += k_f * Math.signum(xVelocity);
    yVelocity += k_f * Math.signum(yVelocity);
    rotVelocity += Constants.DriveConstants.k_rotateF * Math.signum(rotVelocity);
    
    xVelocity = Math.abs(m_currentX - m_xPos) < k_deadzoneMeters ? 0 : xVelocity;
    yVelocity = Math.abs(m_currentY - m_yPos) < k_deadzoneMeters ? 0 : yVelocity;

    m_subsystem.drive(xVelocity, yVelocity, rotVelocity, true, true, new Translation2d(0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, true, false, new Translation2d(0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      (
        Math.abs(m_xPos - m_currentX) < k_deadzoneMeters
        && 
        Math.abs(m_yPos - m_currentY) < k_deadzoneMeters
        &&
        Math.abs(m_rotationDegrees - m_currentRot) < Constants.DriveConstants.k_rotateDeadzone
      )
      ||
      !Constants.States.m_autoRotateAim
    ;
  }
}
