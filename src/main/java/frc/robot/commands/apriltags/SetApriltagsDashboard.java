// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.apriltags;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetApriltagsDashboard extends InstantCommand {
  ApriltagsCamera m_camera;
  ApriltagsCamera m_camera2;
  boolean m_log;

  public SetApriltagsDashboard(ApriltagsCamera camera, ApriltagsCamera camera2, boolean log) {
    Logger.log("SetApriltagsDashboard", 3, "SetApriltagsDashboard()");
    m_camera = camera;
    m_camera2 = camera2;
    m_log = log;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetApriltagsDashboard", 2, String.format("initialize: m_ip=%s, m_log=%b",m_camera.m_ip, m_log));
    m_camera.setDashboard(m_log);
    if (m_camera2 != null) {
      m_camera2.setDashboard(m_log);
    }
  }
}
