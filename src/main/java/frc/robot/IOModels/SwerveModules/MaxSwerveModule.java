package frc.robot.IOModels.SwerveModules;

import static edu.wpi.first.units.Units.*;

import frc.robot.Robot;
import frc.robot.IOModels.Motors.SparkFlexMotor;
import frc.robot.IOModels.Motors.SparkMaxMotor;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.IOModels.Motors.MotorIO.MotorConfig;

/**
 * A swerve module class that corresponds to the prebuilt Rev swerve module
 */
public class MaxSwerveModule implements SwerveModuleIO{
    private static final double wheelRadius = Meters.convertFrom(1.5, Inches);
    private static final double wheelCircumference = 2*Math.PI*wheelRadius;
    private static final double wheelWidth = Meters.convertFrom(1, Inches);
    private static final double steerRatio = 9424.0/203.0; //TODO: Confirm that this is appropriate for your team!
    private static final double driveRatio = 1/5.08;
    private static final double steeringMOI = 1.0/12 * 0.079 * (3*wheelRadius*wheelRadius + wheelWidth*wheelWidth); // 1/12 * M(3R^2+L^2)
    private static final double drivingEncoderPositionFactor = wheelCircumference * driveRatio;

    private final SwerveModuleIO module;

    public MaxSwerveModule(int drivingMotorID, MotorConfig driveConfig, int steerMotorID, MotorConfig steerConfig){
        if(Robot.isSimulation()){
            this.module = new SimulatedMaxSwerveModule(drivingMotorID,driveConfig,steerMotorID,steerConfig);
        } else {
            this.module = new PhysicalMaxSwerveModule(drivingMotorID, driveConfig, steerMotorID, steerConfig);
        }
    }

    private MaxSwerveModule(SwerveModuleIO module){
        this.module = module;
    }
    
    public static MaxSwerveModule getSimulatedModule(int drivingMotorID, MotorConfig driveConfig, int steerMotorID, MotorConfig steerConfig){
        return new MaxSwerveModule(new SimulatedMaxSwerveModule(drivingMotorID,driveConfig,steerMotorID,steerConfig));
    }

    public static MaxSwerveModule getPhysicalModule(int drivingMotorID, MotorConfig driveConfig, int steerMotorID, MotorConfig steerConfig){
        return new MaxSwerveModule(new PhysicalMaxSwerveModule(drivingMotorID, driveConfig, steerMotorID, steerConfig));
    }

    @Override
    public void setSwerveState(SwerveModuleState state){
        this.module.setSwerveState(state);
    }

    @Override
    public void update(){
        this.module.update();
    }

    @Override
    public SwerveModuleState getSwerveModuleState(){
        return this.module.getSwerveModuleState();
    }  

    @Override
    public SwerveModulePosition getSwerveModulePosition(){
        return this.module.getSwerveModulePosition();
    }

    private static class PhysicalMaxSwerveModule implements SwerveModuleIO{
        private SparkFlexMotor drivingMotor;
        private SparkMaxMotor steerMotor;
        public PhysicalMaxSwerveModule(int drivingMotorID, MotorConfig driveConfig, int steerMotorID, MotorConfig steerConfig){
            drivingMotor = new SparkFlexMotor(drivingMotorID, driveConfig);
            steerMotor = new SparkMaxMotor(steerMotorID, steerConfig);
        }

        @Override
        public void setSwerveState(SwerveModuleState state){
            // Cosine compensation
            Rotation2d currentAngle = new Rotation2d(steerMotor.getPosition());
            state.optimize(currentAngle); 
            state.speedMetersPerSecond*=state.angle.minus(currentAngle).getCos();

            steerMotor.setPositionPID(state.angle.getRadians());
            drivingMotor.setVelocityPID(state.speedMetersPerSecond);
        }

        @Override
        public void update(){}

        @Override
        public SwerveModuleState getSwerveModuleState(){
            return new SwerveModuleState(drivingMotor.getVelocity(), new Rotation2d(steerMotor.getPosition()));
        }

        @Override
        public SwerveModulePosition getSwerveModulePosition(){
            return new SwerveModulePosition(drivingMotor.getPosition(), new Rotation2d(steerMotor.getPosition()));
        }
    }

    private static class SimulatedMaxSwerveModule implements SwerveModuleIO{
        private final SwerveModuleSimulation moduleSimulation;
        private final SimulatedMotorController.GenericMotorController driveMotor;
        private final SimulatedMotorController.GenericMotorController steerMotor;
        private final PIDController driveController;
        private final PIDController steerController;

        public SimulatedMaxSwerveModule(int drivingMotorID, MotorConfig driveConfig, int steerMotorID, MotorConfig steerConfig){
            moduleSimulation = new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
                DCMotor.getNeoVortex(1),
                DCMotor.getNeo550(1),
                driveRatio, 
                steerRatio,
                Volts.of(0.1), 
                Volts.of(0.1), 
                Meters.of(wheelRadius),
                KilogramSquareMeters.of(steeringMOI),
                1.6
            ));
            driveMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(steerConfig.currentLimit));
            steerMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(driveConfig.currentLimit));
            driveController = new PIDController(driveConfig.kPIDVel[0], driveConfig.kPIDVel[1], driveConfig.kPIDVel[2]);
            steerController = new PIDController(steerConfig.kPIDVel[0], steerConfig.kPIDVel[1], steerConfig.kPIDVel[2]);
            steerController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void setSwerveState(SwerveModuleState state){
            Rotation2d currentAngle = new Rotation2d(moduleSimulation.getSteerAbsoluteAngle());
            state.optimize(currentAngle); 
            state.speedMetersPerSecond*=state.angle.minus(currentAngle).getCos();

            steerController.setSetpoint(state.angle.getRadians());
            driveController.setSetpoint(state.speedMetersPerSecond);
        }
        @Override
        public void update(){
            SwerveModuleState state = getSwerveModuleState();
            driveMotor.requestVoltage(Volts.of(driveController.calculate(state.speedMetersPerSecond)*RobotController.getBatteryVoltage()));
            steerMotor.requestVoltage(Volts.of(steerController.calculate(state.angle.getRadians())*RobotController.getBatteryVoltage()));
        }

        public SwerveModuleState getSwerveModuleState(){
            return moduleSimulation.getCurrentState();
        }
        public SwerveModulePosition getSwerveModulePosition(){
            return new SwerveModulePosition(moduleSimulation.getDriveWheelFinalPosition().in(Radians)*drivingEncoderPositionFactor,new Rotation2d(moduleSimulation.getSteerAbsoluteAngle()));
        }
    }
}
