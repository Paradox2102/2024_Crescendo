// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FaceSpeaker;
import frc.robot.commands.pivot.SetPivotOffRobotLocation;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShoot extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */
  public AimAndShoot(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new ToggleArcadeDrive(false),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new SetPivotOffRobotLocation(pivotSubsystem),
          new FaceSpeaker(driveSubsystem)
        ),
        new RevCommand(shooterSubsystem, holderSubsystem)
      ),
      new ShootCommand(shooterSubsystem, holderSubsystem)
      //new ToggleArcadeDrive(true),
      // new TestShooter(shooterSubsystem, holderSubsystem, pivotSubsystem, true),
      // new ResetPivot(pivotSubsystem)
    );
  }
}
