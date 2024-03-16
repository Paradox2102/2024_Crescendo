// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveEncoders {
    private final AbsoluteEncoder m_absolute;
    private final RelativeEncoder m_relative;
    private static final double k_relativePosConversionFactor = Constants.DriveConstants.k_turnTicksToRadiansPosition/1920;
    private static final double k_relativeVelConversionFactor = Constants.DriveConstants.k_turnTicksToDegreesVelocity/192;
    private double m_startingOffset;

    public SwerveEncoders(CANSparkMax motor) {
        m_absolute = motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_relative = motor.getEncoder();
    }

    public void initialize() {
        m_relative.setPositionConversionFactor(k_relativePosConversionFactor);
        m_absolute.setPositionConversionFactor(Constants.DriveConstants.k_turnTicksToRadiansPosition);
        m_relative.setVelocityConversionFactor(k_relativeVelConversionFactor);
        m_absolute.setVelocityConversionFactor(Constants.DriveConstants.k_turnTicksToDegreesVelocity);

        setPositionOffset();
    }

    public void setPositionOffset(){
        m_startingOffset = m_absolute.getPosition();
    }

    public double getPosition() {
        return MathUtil.angleModulus(m_relative.getPosition()) + Math.PI + m_startingOffset;
    }

    public double getVelocity() {
        return m_relative.getVelocity();
    }

    public double getAbsolutePosConversionFactor(){
        return m_absolute.getPositionConversionFactor();
    }
}
