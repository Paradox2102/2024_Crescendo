// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aiCamera.AiCamera;
import frc.aiCamera.AiCamera.AiRegion;
import frc.aiCamera.AiCamera.AiRegions;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.PositionTrackerPose;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateAiDistance extends Command {
  DriveSubsystem m_subsystem;
  AiCamera m_camera;
  ApriltagsCamera m_apriltagsCamera;
  PositionTrackerPose m_poseTracker;
  int m_lastFrame = -1;

  /** Creates a new CalibrateAiDistance. */
  public CalibrateAiDistance(DriveSubsystem subsystem, ApriltagsCamera aprilCamera, AiCamera camera, PositionTrackerPose poseTracker) {
    m_subsystem = subsystem;
    m_apriltagsCamera = aprilCamera;
    m_camera = camera;
    m_poseTracker = poseTracker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateAiDistance", 2, "initialize()");
    m_poseTracker.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    m_lastFrame = -1;
    Pose2d pos = m_apriltagsCamera.getPoseAtTime(ApriltagsCamera.getTime() - 0.070);
    Logger.setLogFile("CalibrateAiDistanceLog", "CalAiDist");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AiRegions regions = m_camera.getRegions();
    // Logger.log("CalibrateAiDistance", 1, "execute()");

    if (regions != null && (regions.m_frameNo != m_lastFrame))
    {
      m_lastFrame = regions.m_frameNo;

      AiRegion region = regions.getLargestRegion();

      if (region != null) {
        // Pose2d pose = m_poseTracker.getPose2d();
        Pose2d pose = m_apriltagsCamera.getPoseAtTime(ApriltagsCamera.getTime() - 0.070);  // MUSTFIX - need to get actual delay

        Logger.log("CalibrateAiDistanceLog", 1, String.format(",%f,%f,%f", pose.getTranslation().getX(), region.m_ly, region.m_uy));
      }
    }
    m_subsystem.drive(0.25, 0, 0, true, true, Constants.DriveConstants.k_rotatePoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.closeAllLogFiles();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
