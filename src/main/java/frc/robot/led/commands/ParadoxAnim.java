// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.led.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ParadoxAnim extends Command {
  private final LEDSubsystem m_subsystem;

  private int index = 0;
  private final Timer m_timer = new Timer();
  private int hue = 0;
  private int changeSpeed = 20;
  private final double m_delay;

  /** Creates a new ParadoxAnim. */
  public ParadoxAnim(LEDSubsystem subsystem, double delay) {
    Logger.log("ParadoxAnim", 3, "ParadoxAnim()");
    m_subsystem = subsystem;
    m_delay = delay;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ParadoxAnim", 2, "initialize()");
    m_timer.start();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= m_delay) {
      m_timer.reset();
      index += 1;

      if (index < 30) {
        hue = (hue * changeSpeed + 30 + changeSpeed) / (changeSpeed + 1);
      }

      if (index >= 30 && index < 60) {
        hue = (hue * changeSpeed + 120 + changeSpeed) / (changeSpeed + 1);
      }
      if (index >= 60) {
        hue = (hue * changeSpeed + 180 + changeSpeed) / (changeSpeed + 1);
      }
      if (index > 90) {
        index = 0;
        hue = 0;
      }
      m_subsystem.setAllLEDs(Color.fromHSV(hue, 255, 255));

      m_subsystem.commit();
    }
  }
  /*
   *
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ParadoxAnim", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// index = math.mod(index+1)
