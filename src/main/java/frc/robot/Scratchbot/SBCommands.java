package frc.robot.Scratchbot;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class SBCommands implements SBCommandsInterface{
    public Command moveForwardCommand(double metersToMove){
        //TODO: return a command that will move your robot forward a distance of metersToMove
        //Example: return new GoToPoint(metersToMove, 0, new Rotation2d(0), true);
     }
    public Command moveLeftCommand(double metersToMove){
        //TODO: return a command that will move your robot left a distance of metersToMove
    }
    public Command moveRightCommand(double metersToMove){
        //TODO: return a command that will move your robot right a distance of metersToMove
    }
    public Command moveBackCommand(double metersToMove){
        //TODO: return a command that will move your robot back a distance of metersToMove
    }
    public Command turnClockwiseCommand(double radiansToTurn){
        //TODO: return a command that will turn your robot CW a distance of radiansToTurn
    }
    public Command turnCounterClockwiseCommand(double radiansToTurn){
        //TODO: return a command that will turn your robot CCW a distance of radiansToTurn
    }
    public Command joyStickControlCommand(Supplier<Double> xVelocity, Supplier<Double> yVelocity){
        //TODO: return a command that will allow 2D joystick control
        //Example: return new JoystickControl(xVelocity,yVelocity, () -> {return 0.0;});
    }
}
