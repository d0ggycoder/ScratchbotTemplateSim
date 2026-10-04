package frc.robot.IOModels.Gyros;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class NavX2Gyro implements GyroIO{
    private static NavX2Gyro instance;
    private final GyroIO gyro;
    
    private NavX2Gyro(NavXComType portType){
        if(Robot.isSimulation()){
            gyro = new SimulatedNavX2Gyro(portType);
        } else {
            gyro = new PhysicalNavX2Gyro(portType);
        }
    }

    private NavX2Gyro(GyroIO gyro){
        this.gyro = gyro;
    }

    public static NavX2Gyro getPhysicalGyro(NavXComType portType){
        return new NavX2Gyro(new PhysicalNavX2Gyro(portType));
    }

    public static NavX2Gyro getSimulatedGyro(NavXComType portType){
        return new NavX2Gyro(new SimulatedNavX2Gyro(portType));
    }

    public static NavX2Gyro getDefaultGyro(NavXComType portType){
        if(instance == null){
            instance = new NavX2Gyro(portType);
        }
        return instance;
    }

    @Override
    public void reset(){
        gyro.reset();
    }

    @Override
    public void reset(Rotation2d rotation){
        gyro.reset();
    }

    @Override
    public Rotation2d getRotation(){
        return gyro.getRotation();
    }

    private static class PhysicalNavX2Gyro implements GyroIO{
        private final AHRS gyro;
        private final ReentrantLock mutex = new ReentrantLock();

        public PhysicalNavX2Gyro(NavXComType portType){
            mutex.lock();
            try{
                gyro = new AHRS(portType);
                while(gyro.isCalibrating());
                gyro.reset();
            } finally {
                mutex.unlock();
            }
        }

        @Override
        public void reset(){
            gyro.zeroYaw();
        }

        @Override
        public void reset(Rotation2d angle){
            gyro.setAngleAdjustment(angle.getDegrees()-gyro.getYaw());
        }

        @Override
        public Rotation2d getRotation(){
            return gyro.getRotation2d();
        }
    }

    public static class SimualtedNavX2GyroGetter implements Supplier<GyroSimulation>{
        private final GyroIO gyro;
        public SimualtedNavX2GyroGetter(GyroIO gyro){
            this.gyro = gyro;
        }
        public GyroSimulation get(){
            if(gyro instanceof SimulatedNavX2Gyro){
                return ((SimulatedNavX2Gyro)gyro).gyro;
            } else {
                return null;
            }
        }
    }

    private static class SimulatedNavX2Gyro implements GyroIO{
        private final GyroSimulation gyro;

        public SimulatedNavX2Gyro(NavXComType comType){
            gyro = new GyroSimulation(0, 0);
        }

        @Override
        public void reset(){
            gyro.setRotation(Rotation2d.kZero);
        }

        @Override
        public void reset(Rotation2d angle){
            gyro.setRotation(angle);
        }

        @Override
        public Rotation2d getRotation(){
            return gyro.getGyroReading();
        }
    }
}
