// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class CalibrateShooter extends Command {
  private ManipulatorSubsystem m_backSubsystem;
  /** Creates a new CalibrateShooter. */
  public CalibrateShooter(ManipulatorSubsystem backSubsystem) {
    m_backSubsystem = backSubsystem;
    addRequirements(m_backSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_backSubsystem.setPower(.5);
    m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_speakerShootVelocityRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_backSubsystem.stop();
    System.out.println("CalibrateShooter end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
