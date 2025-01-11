// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.ApriltagsCamera.ApriltagsCameraRegions;
import frc.robot.subsystems.DriveSubsystem;

public class ApriltagAimCommand extends Command {
  private ApriltagsCamera m_camera;
  private DriveSubsystem m_subsystem;
  private ApriltagsCameraRegions m_regions;

  /** Creates a new ApriltagAimCommand. */
  public ApriltagAimCommand(ApriltagsCamera camera, DriveSubsystem subsystem) {
    m_camera = camera;
    m_subsystem =  subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_regions = m_camera.getRegions(0);
    double x = 0;
    double horizDist = 0;
    if (m_regions.getRegion(0) != null) {
      //average the x position of all 4 tag corners to find the center
      double[][] corners = m_regions.getRegion(0).m_corners;
      for (int i = 0; i < 4; i++) {
        x += corners[i][0];
      }
      x /= 4;
      //find horizontal distance, where -0.5 is all the way to the left and 0.5 is all the way to the right
      horizDist = (x/m_regions.m_width)-0.5;
    }
    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag x dist", horizDist);
    m_subsystem.drive(0, horizDist/2, 0, false, false, new Translation2d(0,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
