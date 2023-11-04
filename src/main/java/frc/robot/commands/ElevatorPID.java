package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPID extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private double pidValue, goal;

    public ElevatorPID(ElevatorSubsystem elevator, double goal) {
        elevatorSubsystem = elevator;
        this.goal = goal;

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
        System.out.println("PID Value");
        System.out.println(pidValue);
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
