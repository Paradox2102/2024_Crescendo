// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.AimArcadeDrive;
import frc.robot.commands.gamePieceManipulation.ShootCommand;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassShot extends SequentialCommandGroup {
  /** Creates a new PassShot. */
  DriveSubsystem m_driveSubsystem;
  ManipulatorSubsystem m_manipulatorSubsystem;

  public PassShot(DriveSubsystem driveSubsystem, ManipulatorSubsystem frontSubsystem,
      ManipulatorSubsystem backSubsystem, PivotSubsystem pivotSubsystem, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ToggleShootSideCommand(true),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new SetPivotPos(pivotSubsystem, 70),
                new WaitCommand(0.5),
                new ShootCommand(frontSubsystem, backSubsystem)
            ),
            new AimArcadeDrive(driveSubsystem, () -> controller.getLeftX(), () -> controller.getLeftY())
        )
    );
  }
}
