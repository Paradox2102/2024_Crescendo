package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.apriltagsCamera.ApriltagsCamera;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.led.commands.Blinker;
import frc.robot.led.commands.DisplayCameraError;
import frc.robot.led.commands.ParadoxAnim;
import frc.robot.led.commands.RainbowAnim;
import frc.robot.led.subsystems.LEDSubsystem;

public class LEDConfig {
    private final LEDSubsystem m_string1 = new LEDSubsystem(0, 45);
    private final LEDSubsystem m_string2 = new LEDSubsystem(45, 46, true);
    private final LEDSubsystem m_string3 = new LEDSubsystem(45 + 46, 30);
    private final ApriltagsCamera m_camera;
    private final Robot m_robot;

    private Command m_currentCommand = null;

    @SuppressWarnings("unused")
    private ParallelCommandGroup m_paradox = new ParallelCommandGroup(new ParadoxAnim(m_string1, 0.1),
            new ParadoxAnim(m_string2, 0.1), new ParadoxAnim(m_string3, 0.1));
    private ParallelCommandGroup m_rainbow = new ParallelCommandGroup(new RainbowAnim(m_string1),
            new RainbowAnim(m_string2), new RainbowAnim(m_string3));
    private ParallelCommandGroup m_cameraFailure = new ParallelCommandGroup(new Blinker(m_string1, 0.2, Color.kRed),
            new Blinker(m_string2, 0.2, Color.kRed), new Blinker(m_string3, 0.2, Color.kRed));
    private ParallelCommandGroup m_speaker = new ParallelCommandGroup(new Blinker(m_string1, 0, Color.kPurple),
            new Blinker(m_string2, 0, Color.kPurple), new Blinker(m_string3, 0, Color.kPurple));
    private ParallelCommandGroup m_speakerNote = new ParallelCommandGroup(new Blinker(m_string1, 0.2, Color.kPurple),
            new Blinker(m_string2, 0.2, Color.kPurple), new Blinker(m_string3, 0.2, Color.kPurple));
    private ParallelCommandGroup m_amp = new ParallelCommandGroup(new Blinker(m_string1, 0, Color.kGreen),
            new Blinker(m_string2, 0, Color.kGreen), new Blinker(m_string3, 0, Color.kGreen));
    private ParallelCommandGroup m_ampNote = new ParallelCommandGroup(new Blinker(m_string1, 0.2, Color.kGreen),
            new Blinker(m_string2, 0.2, Color.kGreen), new Blinker(m_string3, 0.2, Color.kGreen));
    private ParallelCommandGroup m_cameraError;

    public LEDConfig(Robot robot, ApriltagsCamera camera) {
        m_robot = robot;
        m_camera = camera;

        m_cameraError = new ParallelCommandGroup(new DisplayCameraError(m_string1, m_camera), new DisplayCameraError(m_string2, m_camera), new DisplayCameraError(m_string3, m_camera));
    }

    public void periodic() {
        Command command = null;

        if (!m_camera.isConnected()&&Constants.States.m_autoRotateAim) {
            command = m_cameraFailure;
        } else if (m_camera.isTagVisible() && m_robot.isDisabled()) {
            // command = m_paradox;
            command = m_cameraError;
        } else if (m_robot.isDisabled()) {
            command = m_rainbow;
        } else if (Constants.States.m_speakerMode) {
            if (Constants.States.m_isGamePieceStowed) {
                command = m_speakerNote;
            } else {
                command = m_speaker;
            }
        } else {
            if (Constants.States.m_isGamePieceStowed) {
                command = m_ampNote;
            } else {
                command = m_amp;
            }
        }

        if (command != m_currentCommand) {
            m_currentCommand = command;
            m_currentCommand.schedule();
        }
    }
}
