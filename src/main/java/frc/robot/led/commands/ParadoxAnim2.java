// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.led.subsystems.LEDSubsystem;

public class ParadoxAnim2 extends Command {
  private final LEDSubsystem m_subsystem;
  private final Timer m_timer = new Timer();
  private int changeSpeed = 20;
  private int index = 0;

  /** Creates a new ParadoxAnim2. */
  public ParadoxAnim2(LEDSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > changeSpeed) {
      m_timer.reset();
      index += 1;
      if (index > 2) {
        index = 0;
      }
      for (var i = index % 3; i < m_subsystem.getSize(); i += 3) {
        m_subsystem.setLED(i, Color.kRed);
      }
      for (var i = (index + 1) % 3; i < m_subsystem.getSize(); i += 3) {
        m_subsystem.setLED(i, Color.kYellow);
      }
      for (var i = (index + 2) % 3; i < m_subsystem.getSize(); i += 3) {
        m_subsystem.setLED(i, Color.kBlue);
      }
      m_subsystem.commit();
    }
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
