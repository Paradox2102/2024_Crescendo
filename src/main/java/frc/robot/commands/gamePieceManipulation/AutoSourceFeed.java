// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.test.D2Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSourceFeed extends ParallelDeadlineGroup {
  /** Creates a new AutoSourceFeed. */
  public AutoSourceFeed(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, ManipulatorSubsystem frontRollers, ManipulatorSubsystem backRollers) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new D2Intake(frontRollers, backRollers, true)
    );
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPivotPos(pivotSubsystem, 0),
      new DriveToPosition(driveSubsystem, null)
    );
  }
}
