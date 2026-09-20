package frc.robot.IOModels.Motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;

public class SparkMaxMotor implements MotorIO{
    private static enum ControlMode{
        kPosition(0),
        kVelocity(1);
        private final int val;
        private final ClosedLoopSlot slot;
        private ControlMode(int val){
            this.val = val;
            this.slot = ClosedLoopSlot.fromInt(val);
        }
    }

    private SparkMax motor;
    private SparkMaxConfig config;
    private RelativeEncoder encoder;
    private ControlMode mode;
    private SparkClosedLoopController controller;
    private int id;

    public SparkMaxMotor(int id, MotorConfig config){
        MotorType motorType = null;
        switch(config.motorType){
            case kBrushed:
                motorType = MotorType.kBrushed;
                break;
            case kBrushless:
                motorType = MotorType.kBrushless;
                break;
        }

        IdleMode idleMode = null;
        switch(config.idleMode){
            case kBrake:
                idleMode = IdleMode.kBrake;
                break;
            case kCoast:
                idleMode = IdleMode.kCoast;
                break; 
        }

        this.id = id;
        this.motor = new SparkMax(id, motorType);
        this.config = new SparkMaxConfig();

        this.config
            .smartCurrentLimit(config.currentLimit)
            .idleMode(idleMode);
        this.config.encoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor);
        this.config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .minOutput(-1, ControlMode.kPosition.slot)
            .maxOutput(1, ControlMode.kPosition.slot)
            .positionWrappingEnabled(config.poseWrapping)
            .pid(
                config.kPIDPos[0],
                config.kPIDPos[1],
                config.kPIDPos[2],
                ControlMode.kPosition.slot
            )
            .feedForward
                .kV(config.kPIDPos[3], ControlMode.kPosition.slot)
                .kS(config.kPIDPos[4], ControlMode.kPosition.slot);
        this.config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .minOutput(-1, ControlMode.kVelocity.slot)
            .maxOutput(1, ControlMode.kVelocity.slot)
            .pid(
                config.kPIDVel[0],
                config.kPIDVel[1],
                config.kPIDVel[2],
                ControlMode.kVelocity.slot
            )
            .feedForward
                .kV(config.kPIDVel[3], ControlMode.kVelocity.slot)
                .kS(config.kPIDVel[4], ControlMode.kVelocity.slot);
        
        this.motor.configure(this.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.encoder = motor.getEncoder();
        this.controller = motor.getClosedLoopController();
    }

    @Override
    public void setVelocityPID(double vel){
        controller.setSetpoint(vel, ControlType.kVelocity, ControlMode.kVelocity.slot);
        mode = ControlMode.kVelocity;
    }

    @Override
    public void setPositionPID(double pos){
        controller.setSetpoint(pos, ControlType.kPosition, ControlMode.kPosition.slot);
        mode = ControlMode.kPosition;
    }

    @Override
    public void setEncoderPos(double pos){
        encoder.setPosition(pos);
    }

    @Override
    public void setVoltage(Voltage volts){
        motor.setVoltage(volts);
    }

    @Override
    public double getVelocity(){
        return encoder.getVelocity();
    }

    @Override
    public double getPosition(){
        return encoder.getPosition();
    }

    @Override
    public double getVoltage(){
        return motor.getAppliedOutput()*motor.getBusVoltage();
    }
}
