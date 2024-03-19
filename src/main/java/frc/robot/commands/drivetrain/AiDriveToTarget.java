// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aiCamera.AiCamera;
import frc.aiCamera.AiCamera.AiRegion;
import frc.aiCamera.AiCamera.AiRegions;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AiDriveToTarget extends Command {
  /** Creates a new AiDriveToTarget. */

  private DriveSubsystem m_subsystem;
  private ApriltagsCamera m_aprilCamera;
  private AiCamera m_aiCamera;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getRot;
  private Translation2d m_targetPos = null;
  private int m_lastFrame = -1;

  public AiDriveToTarget(DriveSubsystem driveSubsystem, ApriltagsCamera aprilCamera, AiCamera aiCamera,
      DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot) {
    m_subsystem = driveSubsystem;
    m_aprilCamera = aprilCamera;
    m_aiCamera = aiCamera;
    m_getX = getX;
    m_getY = getY;
    m_getRot = getRot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("AiDriveToTarget", 0, "initialize");
    m_lastFrame = -1;
    m_targetPos = null;
    m_subsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AiRegions regions = m_aiCamera.getRegions();

    if ((regions != null) && (regions.m_frameNo != m_lastFrame)) {
      AiRegion region = regions.getLargestRegion();
      Pose2d curPos = m_subsystem.getPose();

      m_lastFrame = regions.m_frameNo;

      if (region != null) {
        m_targetPos = region.getPosition(m_aprilCamera, curPos, regions.m_width);
      }
    }

    if (m_targetPos != null) {
      SmartDashboard.putNumber("AI Target X", m_targetPos.getX());
      SmartDashboard.putNumber("AI Target Y", m_targetPos.getY());
    }

    double x = -MathUtil.applyDeadband(m_getX.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double y = -MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double rot = -MathUtil.applyDeadband(m_getRot.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    if (m_subsystem.shouldAim() && rot == 0) {
      rot = MathUtil.applyDeadband(m_subsystem.orientPID(m_subsystem.getFutureRotationalGoalFromTargetDegrees()), 0);
    }

    m_subsystem.drive(
        y,
        x,
        rot,
        true,
        true,
        Constants.DriveConstants.k_rotatePoint);

    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x,
    // y, rot, isFieldRelative));
  }

  // Called once the command ends or is interrupted.
  public void end() {
    Logger.log("AiDriveToTarget", 2, "end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
