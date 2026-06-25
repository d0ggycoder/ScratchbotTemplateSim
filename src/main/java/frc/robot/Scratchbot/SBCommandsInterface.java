package frc.robot.Scratchbot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface SBCommandsInterface {
    public Command moveForwardCommand(double timeToTurnFor);
    public Command moveBackCommand(double timeToTurnFor);
    public Command turnClockwiseCommand(double timeToTurnFor);
    public Command turnCounterClockwiseCommand(double timeToTurnFor);
    public Command joyStickControlCommand(Supplier<Double> xVelocity, Supplier<Double> yVelocity);
}
