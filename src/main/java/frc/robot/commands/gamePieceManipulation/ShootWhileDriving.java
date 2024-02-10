// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetPivotAngleCommand;
import frc.robot.commands.drivetrain.IsInShootingZone;
import frc.robot.commands.pivot.DefaultPivotCommand;
import frc.robot.commands.pivot.ResetPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWhileDriving extends SequentialCommandGroup {
  /** Creates a new ShootWhileDriving. */
  public ShootWhileDriving(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, PivotSubsystem pivotSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {Constants.m_faceSpeaker = true;}),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new IsInShootingZone(driveSubsystem), 
            new RevCommand(shooterSubsystem, holderSubsystem)
          ),
          new ShootCommand(shooterSubsystem, holderSubsystem),
          new InstantCommand(() -> {Constants.m_faceSpeaker = false;})
        ),
        new DefaultPivotCommand(pivotSubsystem, true)
      ),
      new ResetPivot(pivotSubsystem)
    );
  }
}
