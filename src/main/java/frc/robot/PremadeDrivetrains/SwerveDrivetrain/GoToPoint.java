package frc.robot.PremadeDrivetrains.SwerveDrivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToPoint extends Command{
    private static final TrapezoidProfile.State END_ZERO_VEL = new TrapezoidProfile.State(0,0);
    private static final double DIST_TOLERANCE = 0.01;
    private static final double VEL_TOLERANCE = 0.01;

    private Pose2d targetPose;
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedSupplier;

    private PIDController distController = new PIDController(0, 0, 0);
    private TrapezoidProfile speedProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxAcceleration));
    private TrapezoidProfile.State initialSpeedState;

    private double startTime;
    public GoToPoint(Pose2d targetPose, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier){
        this.targetPose = targetPose;
        this.poseSupplier = poseSupplier;
        this.speedSupplier= speedSupplier;

        initialSpeedState = getSpeedState();
    }

    private double getDistance(){
        return targetPose.minus(poseSupplier.get()).getTranslation().getNorm();
    }

    private double getSpeed(){
        ChassisSpeeds speeds = speedSupplier.get();
        return speeds.vxMetersPerSecond*speeds.vxMetersPerSecond+speeds.vyMetersPerSecond*speeds.vyMetersPerSecond;
    }

    private TrapezoidProfile.State getSpeedState(){
        return new TrapezoidProfile.State(getDistance(),getSpeed());
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State calculatedSpeedState = speedProfile.calculate(
            Timer.getFPGATimestamp()-startTime,initialSpeedState,END_ZERO_VEL);
        double targetVel = calculatedSpeedState.velocity+distController.calculate(getDistance(),calculatedSpeedState.position);

        Translation2d delta = targetPose.minus(poseSupplier.get()).getTranslation();
        Translation2d setVel = delta.times(targetVel/delta.getNorm());
        Drivetrain.getInstance().setSwerveStates(setVel.getX(), setVel.getY(), 0, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return getSpeed() < VEL_TOLERANCE && getDistance() < DIST_TOLERANCE;
    }
}
