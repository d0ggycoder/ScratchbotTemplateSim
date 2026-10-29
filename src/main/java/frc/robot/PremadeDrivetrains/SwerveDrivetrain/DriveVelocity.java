package frc.robot.PremadeDrivetrains.SwerveDrivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveVelocity extends Command{
    private static final double DEADBAND = 0.05;

    Supplier<Double> velX;
    Supplier<Double> velY;
    Supplier<Double> omega;
    boolean fieldRelative;
    Drivetrain swerveDrive;

    public DriveVelocity(Supplier<Double> velX, Supplier<Double> velY, Supplier<Double> omega){
        this(velX,velY,omega,true);
    }

    public DriveVelocity(Supplier<Double> velX, Supplier<Double> velY, Supplier<Double> omega, boolean fieldRelative){
        this.omega = omega;
        this.velX = velX;
        this.velY = velY;
        this.fieldRelative = fieldRelative;
        this.swerveDrive = Drivetrain.getInstance();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = velX.get();
        double adjX = (x*x)<DEADBAND?0:Math.signum(x)*x*x;
        double y = velY.get();
        double adjY = (y*y)<DEADBAND?0:Math.signum(y)*y*y;
        double o = omega.get();
        double adjO = (o*o)<DEADBAND?0:Math.signum(o)*o*o;
        
        swerveDrive.setSwerveStates(adjX, adjY, adjO, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setSwerveStates(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
