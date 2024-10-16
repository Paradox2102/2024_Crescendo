// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StopEverything;
import frc.robot.commands.ToggleAutoAim;
import frc.robot.commands.ToggleShootSideCommand;
import frc.robot.commands.drivetrain.JukeShotRotateAim;
import frc.robot.commands.pivot.DefaultPivotCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JukeShot extends SequentialCommandGroup {
  /** Creates a new JukeShot. */
  public JukeShot(PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, boolean rotateLeft) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleAutoAim(),
      new ToggleShootSideCommand(false),
      new StopEverything(driveSubsystem, frontSubsystem, backSubsystem),
      new ParallelDeadlineGroup(
        new JukeShotRotateAim(driveSubsystem, rotateLeft), 
        new DefaultPivotCommand(pivotSubsystem, driveSubsystem, true),
        new DefaultManipulatorCommand(backSubsystem, driveSubsystem, false),
        new DefaultManipulatorCommand(frontSubsystem, driveSubsystem, true)
      ),
      new ParallelDeadlineGroup(
        new ShootCommand(frontSubsystem, backSubsystem), 
        new DefaultPivotCommand(pivotSubsystem, driveSubsystem, true),
        new RunCommand(() -> {driveSubsystem.drive(0, 0, 0, true, true, new Translation2d(0, 0));}, driveSubsystem)
      ),
      new ToggleShootSideCommand(true)
    );
  }
}
