// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.led.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ProgressBar extends Command {
  private final LEDSubsystem m_subsystem;
  private int m_remaining;
  private final Timer m_timer = new Timer();
  private double m_time;

  /** Creates a new SpeakerAnim. */
  public ProgressBar(LEDSubsystem subsystem, double time) {
    Logger.log("ProgressBar", 3, "ProgressBar()");
    m_subsystem = subsystem;
    m_time = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ProgressBar", 2, "initialize()");
    m_timer.start();
    m_timer.reset();
    m_remaining = m_subsystem.getSize();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int size = m_subsystem.getSize();
    m_remaining = size - (int) (size * m_timer.get() / m_time);
    int green = 255 * m_remaining / size;
    int red = 255 - green;

    m_subsystem.setAllLEDs(Color.kBlack);

    for (int i = 0; i < m_remaining; i++) {
      m_subsystem.setLED(i, new Color(red, green, 0));
    }

    m_subsystem.commit();
  }
  /*
   *
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ProgressBar", 2, String.format("end(%b)", interrupted));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_remaining < 0;
  }
}

// index = math.mod(index+1)
