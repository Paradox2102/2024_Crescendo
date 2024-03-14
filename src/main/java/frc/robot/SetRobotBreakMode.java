// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.StickSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRobotBreakMode extends InstantCommand {
  Trigger m_brake;
  DriveSubsystem m_driveSubsystem;
  PivotSubsystem m_pivotSubsystem;
  ManipulatorSubsystem m_shooterSubsystem;
  ManipulatorSubsystem m_holderSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  StickSubsystem m_stickSubsystem;

  public SetRobotBreakMode(Trigger brake, DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem, ElevatorSubsystem elevatorSubsystem, StickSubsystem stickSubsystem) {
    m_brake = brake;

    m_driveSubsystem = driveSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_stickSubsystem = stickSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean brake = m_brake.getAsBoolean();
    m_driveSubsystem.setBrakeMode(brake);
    m_pivotSubsystem.setBrakeMode(brake);
    m_shooterSubsystem.setBrakeMode(brake);
    m_holderSubsystem.setBrakeMode(brake);
    m_stickSubsystem.setBrakeMode(brake);
  }
}
