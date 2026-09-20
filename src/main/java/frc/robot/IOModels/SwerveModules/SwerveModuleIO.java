package frc.robot.IOModels.SwerveModules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    public void setSwerveState(SwerveModuleState state);
    public void update();

    public SwerveModuleState getSwerveModuleState();
    public SwerveModulePosition getSwerveModulePosition();
}
