package frc.robot.IOModels.Gyros;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    public void reset();
    public void reset(Rotation2d angle);
    public Rotation2d getRotation();
}
