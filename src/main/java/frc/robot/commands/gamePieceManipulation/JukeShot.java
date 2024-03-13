// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ToggleShootSideCommand;
import frc.robot.commands.drivetrain.JukeShotRotateAim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JukeShot extends SequentialCommandGroup {
  /** Creates a new JukeShot. */
  public JukeShot(DriveSubsystem driveSubsystem, ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem, boolean rotateLeft) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleShootSideCommand(false),
      new ParallelDeadlineGroup(
        new JukeShotRotateAim(driveSubsystem, rotateLeft), 
        new WaitCommand(1),
        new DefaultManipulatorCommand(holderSubsystem, driveSubsystem, false),
        new DefaultManipulatorCommand(shooterSubsystem, driveSubsystem, true)
      ),
      new ShootCommand(shooterSubsystem, holderSubsystem),
      new ToggleShootSideCommand(true)
    );
  }
}
