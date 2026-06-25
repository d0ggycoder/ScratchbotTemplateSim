package frc.robot.Scratchbot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Lab.Lab;

public class Scratchbot{
    private static SequentialCommandGroup toRun = new SequentialCommandGroup();
    public static SBCommands sbCommands = new SBCommands();
    /**
     * Moves the robot forward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot forward.
     */
    public static void sb_moveForward(double metersToMove){
        toRun.addCommands(sbCommands.moveForwardCommand(metersToMove));
    }
    /**
     * Moves the robot left by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot left.
     */
    public static void sb_moveLeft(double metersToMove){
        toRun.addCommands(sbCommands.moveLeftCommand(metersToMove));
    }
    /**
     * Moves the robot right by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot right.s
     */
    public static void sb_moveRight(double metersToMove){
        toRun.addCommands(sbCommands.moveRightCommand(metersToMove));
    }
    /**
     * Moves the robot backward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot backward.
     */
    public static void sb_moveBackward(double metersToMove){
        toRun.addCommands(sbCommands.moveBackCommand(metersToMove));
    }
    /**
     * Turns the robot clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot clockwise.
     */
    public static void sb_turnClockwise(double radiansToTurn){
        toRun.addCommands(sbCommands.turnClockwiseCommand(radiansToTurn));
    }
    /**
     * Turns the robot counter-clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot counter-clockwise.
     */
    public static void sb_turnCounterClockwise(double radiansToTurn){
        toRun.addCommands(sbCommands.turnCounterClockwiseCommand(radiansToTurn));
    }
    /**
     * @return The value of the left joystick's X axis of the controller.
     */
    public static double sb_getControllerLeftX(){
        return Robot.controller.getLeftX();
    }
    /**
     * @return The value of the left joystick's Y axis of the controller.
     */
    public static double sb_getControllerLeftY(){
        return Robot.controller.getLeftY();
    }
    /**
     * @return The value of the right joystick's X axis of the controller.
     */
    public static double sb_getControllerRightX(){
        return Robot.controller.getRightX();
    }
    /**
     * @return The value of the right joystick's Y axis of the controller.
     */
    public static double sb_getControllerRightY(){
        return Robot.controller.getRightY();
    }
    /**
     * Sets the robot's movement using joystick control (or any other suppliers). 
     * @param xVelocity Supplier for the X velocity of the robot. [-1,1]
     * @param yVelocity Supplier for the Y velocity of the robot. [-1,1]
     */
    public static void sb_setRobotMovement(Supplier<Double> xVelocity, Supplier<Double> yVelocity){
        toRun.addCommands(sbCommands.joyStickControlCommand(xVelocity, yVelocity));
    }

    public static SequentialCommandGroup getToRun(){
        return toRun;
    }
    public static void clearCommands(){
        toRun = new SequentialCommandGroup();
    }
    public static void initializeLab() {
        Lab.runLab();
        runScratchbotCommands();
    }
    public static void runScratchbotCommands(){
        CommandScheduler.getInstance().cancelAll();
        SequentialCommandGroup sequence = Scratchbot.getToRun();
        CommandScheduler.getInstance().schedule(sequence);
        Scratchbot.clearCommands();
    }
}
