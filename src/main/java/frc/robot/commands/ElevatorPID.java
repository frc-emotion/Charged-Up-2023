package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPID extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private double pidValue, goal;

    public ElevatorPID(ElevatorSubsystem elevator, double goal) {
        elevatorSubsystem = elevator;
        this.goal = goal; // Set the goal of elevator position

        addRequirements(elevatorSubsystem);
        
    }

    @Override
    public void initialize() {
        elevatorSubsystem.resetHeight();
        elevatorSubsystem.convertToMeters();

    }

    @Override
    public void execute() {
        //elevatorSubsystem.setElevatorGoal(goal);
       // pidValue = elevatorSubsystem.getPIDOutputVoltsAuto();
        // System.out.println("PID Value");
        // System.out.println(pidValue);
        //elevatorSubsystem.setElePID(elevatorSubsystem.getPIDOutputVolts(elevatorSubsystem.getHeight()));
        // feedForwardVal = feedForward.calculate(0.02);
        // MathUtil.clamp(pidValue, 0, 12);
       elevatorSubsystem.runElevatorAuto(goal);
    }

    @Override
    public void end(boolean interrupted) {
       elevatorSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
