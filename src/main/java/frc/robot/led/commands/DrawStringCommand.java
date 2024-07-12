// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.apriltagsCamera.Logger;
import frc.robot.led.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrawStringCommand extends InstantCommand {
  private final LEDSubsystem m_subsystem;
  private final String m_text;
  private final Color m_fgColor;
  private final Color m_bgColor;

  public DrawStringCommand(LEDSubsystem subsystem, String text, Color fgColor, Color bgColor) {
    Logger.log("DrawStringCommand", 3, "DrawStringCommand()");
    m_subsystem = subsystem;
    m_text = text;
    m_fgColor = fgColor;
    m_bgColor = bgColor;

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
    Logger.log("DrawStringCommand", 2, "initialize()");
    m_subsystem.drawString(m_text, 0, 0, m_fgColor, m_bgColor);
    m_subsystem.commit();
  }
}
