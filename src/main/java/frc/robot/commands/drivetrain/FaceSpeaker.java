// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FaceSpeaker extends Command {
  /** Creates a new FaceSpeaker. */
  DriveSubsystem m_subsystem;
  public FaceSpeaker(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0, 0, m_subsystem.orientPID(m_subsystem.getRotationalDistanceFromSpeakerDegrees()), true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.orientPID(m_subsystem.getRotationalDistanceFromSpeakerDegrees())) < Constants.DriveConstants.k_rotateDeadzone;
  }
}