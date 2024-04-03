package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroInterface {

    static public class GyroPidgen extends Pigeon2 implements GyroInterface {

        public GyroPidgen(int deviceId) {
            super(deviceId);
        }

    }

    public void reset();
    public double getAngle();
    public Rotation2d getRotation2d();
    public double getRate();
}
