// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;
import frc.apriltagsCamera.ApriltagsCamera.ApriltagsCameraStats;
import frc.robot.led.subsystems.LEDSubsystem;

public class DisplayCameraError extends Command {
  private final LEDSubsystem m_subsystem;
  private final ApriltagsCamera m_camera;
  private final double k_minError = 20.0;

  /** Creates a new DisplayCameraError. */
  public DisplayCameraError(LEDSubsystem subsystem, ApriltagsCamera camera) {
    Logger.log("DisplayCameraError", 3, "DisplayCameraError()");
    m_subsystem = subsystem;
    m_camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DisplayCameraError", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ApriltagsCameraStats stats = m_camera.getStats();

    m_subsystem.setAllLEDs(Color.kRed);

    if (stats.m_yawError < k_minError)
    {
      int length = m_subsystem.getSize() - (int) (stats.m_yawError * m_subsystem.getSize() / k_minError);

      m_subsystem.setLEDs(0, length, Color.kGreen);
    }

    m_subsystem.commit();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DisplayCameraError", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
