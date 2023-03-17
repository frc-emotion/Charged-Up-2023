package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SetHeightCommand extends CommandBase{
   /*  private Elevator myElevator;

    public SetHeightCommand(Elevator elevator){
        myElevator = elevator;
        addRequirements(myElevator);
    }
    
    @Override
    public void initialize(){}


    @Override
    public void execute(){
        myElevator.setHeight();
    }

    @Override
    public void end(boolean interrupted){
        myElevator.stopElevator();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    /* */
}
