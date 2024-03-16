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
public class SetApriltagsLogging extends InstantCommand {
  ApriltagsCamera m_camera;
  boolean m_log;

  public SetApriltagsLogging(ApriltagsCamera camera, ApriltagsCamera sideCamera, boolean log) {
    Logger.log("SetApriltagsLogging", 3, "SetApriltagsLogging()");
    m_camera = camera;
    m_log = log;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetApriltagsLogging", 2, "initialize()");
    m_camera.setLogging(m_log);
  }
}
