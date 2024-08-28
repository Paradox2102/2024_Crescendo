// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.apriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotOffRobotLocation extends InstantCommand {
  PivotSubsystem m_subsystem;
  DriveSubsystem m_driveSubsystem;
  public SetPivotOffRobotLocation(PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem) {
    m_subsystem = pivotSubsystem;
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = m_subsystem.getPivotAngleFromDistanceFromSpeaker(m_driveSubsystem.getTranslationalDistanceFromSpeakerMeters());
    m_subsystem.setPositionDegrees(angle);
    // System.out.println("ksdufkdsfkjdshfkdsfhkdsjhfds");
    // System.out.println(angle);
    // System.out.println(m_driveSubsystem.getTranslationalDistanceFromSpeakerMeters());
  }
}
