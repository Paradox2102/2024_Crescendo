// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.IsInPassingZone;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSourceFeedPassCycle extends SequentialCommandGroup {
  /** Creates a new AutoSourceFeedPassCycle. */
  public AutoSourceFeedPassCycle(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, ManipulatorSubsystem frontRollers, ManipulatorSubsystem backRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RepeatCommand(
        new SequentialCommandGroup(
          new AutoSourceFeed(driveSubsystem, pivotSubsystem, frontRollers, backRollers),
          new ParallelDeadlineGroup(
            // This command consists solely of an isFinished() method.  It should have been a call to until() instead. -Gavin
            new IsInPassingZone(driveSubsystem),
            // BUG: This -1 here moves us in the positive X direction, regardless of alliance colour. -Gavin
            new ArcadeDrive(driveSubsystem, () -> -1, () -> 0, () -> 0)
          ),
          new ShootCommand(frontRollers, backRollers)
          
        )
      )
      
    );
  }
}
