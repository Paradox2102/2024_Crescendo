// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

/** Add your docs here. */
public class SwerveEncoders {
    private static final double k_ticksToDegrees = 48/360;
    AbsoluteEncoder m_absolute;
    RelativeEncoder m_relative;
    public SwerveEncoders(CANSparkMax motor) {
        m_absolute = motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_relative = motor.getEncoder();
    }

    public void initialize() {
        m_relative.setPosition(m_absolute.getPosition());
    }

    public double getRelativePosInDegrees() {
        return m_relative.getPosition() * k_ticksToDegrees;
    }
}
