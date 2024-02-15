// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.apriltagsCamera.PositionServer;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.ApriltagsCamera.ApriltagsCameraType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPickUpGamePiece;
import frc.robot.commands.DisableEverything;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetPivotAngleCommand;
import frc.robot.commands.SetSpeakerAmpMode;
import frc.robot.commands.ToggleShootSideCommand;
import frc.robot.commands.apriltags.SetApriltagsDashboard;
import frc.robot.commands.apriltags.SetApriltagsLogging;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.AutoOrientCommand;
import frc.robot.commands.gamePieceManipulation.AimAndShoot;
import frc.robot.commands.gamePieceManipulation.CheckIntakeStowed;
import frc.robot.commands.gamePieceManipulation.FeedCommand;
import frc.robot.commands.gamePieceManipulation.IntakeCommand;
import frc.robot.commands.gamePieceManipulation.RevCommand;
import frc.robot.commands.gamePieceManipulation.ShootCommand;
import frc.robot.commands.pivot.DefaultPivotCommand;
import frc.robot.commands.pivot.SetPivotOffRobotLocation;
import frc.robot.commands.test.D2Intake;
import frc.robot.commands.test.IncrementPivotCommand;
import frc.robot.commands.test.SlowTurn;
import frc.robot.commands.test.TestPivot;
import frc.robot.commands.test.TestShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSensors;
import frc.robot.subsystems.ShooterSubsystem;
import frc.triggers.ToggleTrigger;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ApriltagsCamera m_apriltagCamera = new ApriltagsCamera();
  PositionServer m_posServer = new PositionServer(m_apriltagCamera);

  private final ShooterSensors m_shooterSensors = new ShooterSensors();
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_apriltagCamera);
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(m_driveSubsystem);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final HolderSubsystem m_holderSubsystem = new HolderSubsystem();

  private final CommandJoystick m_joystick = new CommandJoystick(1);
  public final PositionTrackerPose m_tracker = new PositionTrackerPose(m_posServer, 0, 0, m_driveSubsystem);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setTracker(m_tracker);
    NamedCommands.registerCommand("shoot", new ShootCommand(m_shooterSubsystem, m_holderSubsystem));
    NamedCommands.registerCommand("intake", new IntakeCommand(m_holderSubsystem, m_shooterSubsystem, m_pivotSubsystem));
    NamedCommands.registerCommand("rev shooter", new RevCommand(m_shooterSubsystem, m_holderSubsystem));
    NamedCommands.registerCommand("aim", new SetPivotOffRobotLocation(m_pivotSubsystem));
    NamedCommands.registerCommand("switch sides", new ToggleShootSideCommand(false));
    NamedCommands.registerCommand("set speaker", new SetSpeakerAmpMode(true));
    NamedCommands.registerCommand("feedthrough", new CheckIntakeStowed());
    autoChooser = AutoBuilder.buildAutoChooser();

    m_apriltagCamera.setCameraInfo(5.125, 15.5, 0, ApriltagsCameraType.GS_6mm); // y = 9.5
    m_apriltagCamera.connect("10.21.2.11", 5800);

    m_posServer.start();
    SmartDashboard.putData("Auto To Run",autoChooser);
  }

  private boolean getPositionServerButtonState(int button) {
    return m_posServer.getButtonState(button);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(() -> getPositionServerButtonState(1)).onTrue(new SetApriltagsLogging(m_apriltagCamera, true));
    new Trigger(() -> getPositionServerButtonState(2)).onTrue(new SetApriltagsLogging(m_apriltagCamera, false));
    new Trigger(() -> getPositionServerButtonState(3)).onTrue(new SetApriltagsDashboard(m_apriltagCamera, true));
    new Trigger(() -> getPositionServerButtonState(4)).onTrue(new SetApriltagsDashboard(m_apriltagCamera, false));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getRightX()
    ));

    m_pivotSubsystem.setDefaultCommand(new DefaultPivotCommand(m_pivotSubsystem, false));


    m_driverController.rightBumper().onTrue(new InstantCommand(() -> {Constants.m_faceSpeaker = !Constants.m_faceSpeaker;}));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.leftTrigger().toggleOnTrue(new ShootCommand(m_shooterSubsystem, m_holderSubsystem));
    // m_driverController.leftTrigger().toggleOnTrue(new AimAndShoot(m_pivotSubsystem, m_shooterSubsystem, m_holderSubsystem, m_driveSubsystem));
    // m_driverController.leftBumper().toggleOnTrue(new SetPivotAngleCommand(m_pivotSubsystem, 20.9));
    m_driverController.leftBumper().onTrue(new SetPivotOffRobotLocation(m_pivotSubsystem));
    // m_driverController.rightTrigger().whileTrue(new AutoPickUpGamePiece(m_driveSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_holderSubsystem, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX()));
    m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_holderSubsystem, m_shooterSubsystem, m_pivotSubsystem));
    
    m_driverController.y().onTrue(new AutoOrientCommand(m_driveSubsystem, 180, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.a().onTrue(new AutoOrientCommand(m_driveSubsystem, 0, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.b().onTrue(new AutoOrientCommand(m_driveSubsystem, -90, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.x().onTrue(new AutoOrientCommand(m_driveSubsystem, 90, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));

    m_driverController.povUp().onTrue(new ResetGyro(m_driveSubsystem, 180));
    m_driverController.povDown().onTrue(new ResetGyro(m_driveSubsystem, 0));
    m_driverController.povRight().onTrue(new ResetGyro(m_driveSubsystem, 90));
    m_driverController.povLeft().onTrue(new ResetGyro(m_driveSubsystem, -90));

    ToggleTrigger shootIntake = new ToggleTrigger(m_joystick.button(7));

    m_joystick.button(1).toggleOnTrue(new RevCommand(m_shooterSubsystem, m_holderSubsystem));
    m_joystick.button(2).whileTrue(new D2Intake(m_shooterSubsystem, m_holderSubsystem, true));
    m_joystick.button(6).onTrue(new IncrementPivotCommand(m_pivotSubsystem, true));
    m_joystick.button(4).onTrue(new IncrementPivotCommand(m_pivotSubsystem, false));
    m_joystick.button(5).onTrue(new TestPivot(m_pivotSubsystem, -24));
    m_joystick.button(7).onTrue(new ToggleShootSideCommand(shootIntake.getAsBoolean()));
    m_joystick.button(8).toggleOnTrue(new FeedCommand(m_shooterSubsystem, m_holderSubsystem));
    // m_joystick.button(11).toggleOnTrue(new ShootWhileDriving(m_driveSubsystem, m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem));

    m_joystick.button(11).onTrue(new DisableEverything(m_driveSubsystem, m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem));
    m_joystick.button(12).whileTrue(new SlowTurn(m_driveSubsystem, m_apriltagCamera));
  }

  public double getThrottle() {
    return m_joystick.getThrottle();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
