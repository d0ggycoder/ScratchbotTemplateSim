package frc.robot.IOModels.Motors;

import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    public void setVelocityPID(double vel);
    public void setPositionPID(double pos);
    public void setEncoderPos(double pos);
    public void setVoltage(Voltage volts);

    public double getVelocity();
    public double getPosition();
    public double getVoltage();

    public static class MotorConfig{
        public static enum IdleMode{
            kCoast,
            kBrake;
        }
        public static enum MotorType{
            kBrushed,
            kBrushless;
        }
        //Order: kP, kI, kD, kV, kS
        public double[] kPIDPos = new double[5]; 
        public double[] kPIDVel = new double[5];
        public boolean poseWrapping = true;
        public double wrapLow = 0;
        public double wrapHigh = 2*Math.PI;
        public boolean reversed = false;
        public IdleMode idleMode = IdleMode.kCoast;
        public double positionConversionFactor = 1;
        public double velocityConversionFactor = 1;
        public MotorType motorType;
        public int currentLimit = 60;
        
        public MotorConfig(){}

        /**
         * Sets the positional PID coefficients of this configuration object <p>
         * Note that it's better not to do this from an array where possible to prevent ambiguity; use 
         * @param pidArr Array of PID coefficients in the order of [kP,kI,kD,kV,kS]
         * @return This object for method chaining
         */
        public MotorConfig withPositionPID(double[] pidArr){
            kPIDPos[0] = pidArr[0];
            kPIDPos[1] = pidArr[1];
            kPIDPos[2] = pidArr[2];
            if(pidArr.length >= 4) kPIDPos[3] = pidArr[3];
            if(pidArr.length >= 5) kPIDPos[4] = pidArr[4];
            return this;
        }

        /**
         * Sets the positional PID coefficients of this configuration object
         * @param kP Proportional control gain 
         * @param kI Integral control gain 
         * @param kD Derivative control gain
         * @return This object for method chaining
         */
        public MotorConfig withPositionPID(double kP, double kI, double kD){
            kPIDPos[0] = kP;
            kPIDPos[1] = kI;
            kPIDPos[2] = kD;
            return this;
        }

        /**
         * Sets the positional PID coefficients of this configuration object
         * @param kP Proportional control gain 
         * @param kI Integral control gain 
         * @param kD Derivative control gain
         * @param kS Static friction control gain
         * @param kV Cruise velocity voltage control gain
         * @return This object for method chaining
         */
        public MotorConfig withPositionPID(double kP, double kI, double kD, double kS, double kV){
            kPIDPos[0] = kP;
            kPIDPos[1] = kI;
            kPIDPos[2] = kD;
            kPIDPos[3]= kS;
            kPIDPos[4] = kV;
            return this;
        }

        /**
         * Sets the velocity-related PID coefficients of this configuration object <p>
         * Note that it's better not to do this from an array where possible to prevent ambiguity; use 
         * @param pidArr Array of PID coefficients in the order of [kP,kI,kD,kV,kS]
         * @return This object for method chaining
         */
        public MotorConfig withVeloctiyPID(double[] pidArr){
            kPIDVel[0] = pidArr[0];
            kPIDVel[1] = pidArr[1];
            kPIDVel[2] = pidArr[2];
            if(pidArr.length >= 4) kPIDVel[3] = pidArr[3];
            if(pidArr.length >= 5) kPIDVel[4] = pidArr[4];
            return this;
        }

        /**
         * Sets the velocity-related PID coefficients of this configuration object
         * @param kP Proportional control gain 
         * @param kI Integral control gain 
         * @param kD Derivative control gain
         * @return This object for method chaining
         */
        public MotorConfig withVelocityPID(double kP, double kI, double kD){
            kPIDVel[0] = kP;
            kPIDVel[1] = kI;
            kPIDVel[2] = kD;
            return this;
        }

        /**
         * Sets the velocity-related PID coefficients of this configuration object
         * @param kP Proportional control gain 
         * @param kI Integral control gain 
         * @param kD Derivative control gain
         * @param kS Static friction control gain
         * @param kV Cruise velocity voltage control gain
         * @return This object for method chaining
         */
        public MotorConfig withVelocityPID(double kP, double kI, double kD, double kS, double kV){
            kPIDVel[0] = kP;
            kPIDVel[1] = kI;
            kPIDVel[2] = kD;
            kPIDVel[3]= kS;
            kPIDVel[4] = kV;
            return this;
        }

        public MotorConfig withPositionConversionFactor(double positionConversionFactor){
            this.positionConversionFactor = positionConversionFactor;
            return this;
        }

        public MotorConfig withVelocityConversionFactor(double velocityConversionFactor){
            this.velocityConversionFactor = velocityConversionFactor;
            return this;
        }

        public MotorConfig withIdleMode(IdleMode idleMode){
            this.idleMode = idleMode;
            return this;
        }

        public MotorConfig withPoseWrapping(boolean poseWrapping){
            this.poseWrapping = poseWrapping;
            return this;
        }

        public MotorConfig withReversed(boolean reversed){
            this.reversed = reversed;
            return this;
        }
    }
}
