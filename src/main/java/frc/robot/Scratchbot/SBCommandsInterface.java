package frc.robot.Scratchbot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface SBCommandsInterface {
    public Command moveForwardCommand(double metersToMove);
    public Command moveLeftCommand(double metersToMove);
    public Command moveRightCommand(double metersToMove);
    public Command moveBackCommand(double metersToMove);
    public Command turnClockwiseCommand(double radiansToTurn);
    public Command turnCounterClockwiseCommand(double radiansToTurn);
    public Command joyStickControlCommand(Supplier<Double> xVelocity, Supplier<Double> yVelocity);
}
