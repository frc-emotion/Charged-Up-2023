package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.mace.Mace;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class ManualControlElevator extends CommandBase{
    private ElevatorSubsystem myElevator;
    private Supplier<Double> elevatorSpeedFunc;
    private Mace mSub;
    public ManualControlElevator(ElevatorSubsystem elevator,Supplier<Double> func){
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