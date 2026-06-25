package frc.robot.Scratchbot;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class SBCommands implements SBCommandsInterface{
    public Command moveForwardCommand(double timeToMoveFor){
        //TODO: return a command that will move your robot forward for timeToMoveFor seconds
        //Example: return new GoToPoint(timeToMoveFor, 0, new Rotation2d(0), true);
     }
    public Command moveBackCommand(double timeToMoveFor){
        //TODO: return a command that will move your robot backward for timeToMoveFor seconds
    }
    public Command turnClockwiseCommand(double timeToTurnFor){
        //TODO: return a command that will turn your robot CW for timeToTurnFor seconds.
    }
    public Command turnCounterClockwiseCommand(double timeToTurnFor){
        //TODO: return a command that will turn your robot CCW for timeToTurnFor seconds.
    }
    public Command joyStickControlCommand(Supplier<Double> xInput, Supplier<Double> yInput){
        //TODO: return a command that will allow 2D joystick control
        //The robot should steer left when yInput is positive and steer right when yInput is negative. Forward when xInput is positive.
        /*Example:
         return new ArcadeControl(xInput,yInput);
        */
    }
}
