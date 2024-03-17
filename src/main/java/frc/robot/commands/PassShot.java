// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassShot extends SequentialCommandGroup {
  /** Creates a new PassShot. */
  DriveSubsystem m_driveSubsystem;
  ManipulatorSubsystem m_manipulatorSubsystem;
  public PassShot(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_manipulatorSubsystem = manipulatorSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
