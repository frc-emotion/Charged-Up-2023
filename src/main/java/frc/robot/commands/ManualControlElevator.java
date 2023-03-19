package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class ManualControlElevator extends CommandBase{
    private Elevator myElevator;
    private Supplier<Double> elevatorSpeedFunc;

    public ManualControlElevator(Elevator elevator, Supplier<Double> func){
        myElevator = elevator;
        elevatorSpeedFunc = func;
        addRequirements(myElevator);
    }

    @Override
    public void initialize(){}

    
    @Override
    public void execute(){
        myElevator.manualMove(elevatorSpeedFunc);
    }

    @Override
    public void end(boolean interrupted){
        myElevator.stopElevator();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}