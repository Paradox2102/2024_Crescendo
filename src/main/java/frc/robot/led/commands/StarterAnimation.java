// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.led.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class StarterAnimation extends Command {
  private final LEDSubsystem m_subsystem;
  private final Color m_color;

 
  private int index = 0;
  private final Timer m_timer = new Timer();
  private  Color m_bColor;
  private  double m_delay;
  private  int m_length;

  /** Creates a new IntakeAnimation. */
  public StarterAnimation(LEDSubsystem subsystem, Color color, Color backgColor, Double delay, int LEDLength) {
    Logger.log("IntakeAnimation", 3, "IntakeAnimation()");
    m_subsystem = subsystem;
    m_color = color;
    m_bColor = backgColor;
    m_delay = delay;
    m_length = LEDLength;
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("IntakeAnimation", 2, "initialize()");
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
       
      if (index >= m_subsystem.getSize()){
        index = 1-m_length;
      }
      m_subsystem.setAllLEDs(m_bColor);
      m_subsystem.setLEDsMiddlePeak(index, m_length, m_color, 0.1);
      m_subsystem.commit();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("IntakeAnimation", 2, String.format("end(%b)", interrupted));
    index=0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// index = math.mod(index+1)
