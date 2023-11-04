package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmPID extends CommandBase {

    private ArmSubsystem aSubsystem;

    private double pidValue, goal;

    public ArmPID(ArmSubsystem arm, double goal) {
        aSubsystem = arm;

        this.goal = goal;
        addRequirements(aSubsystem);

        
    }

    @Override
    public void initialize() {
        //elevatorSubsystem.resetHeight();

        //elevatorSubsystem.convertToMeters();

        
    }

    @Override
    public void execute() {
      aSubsystem.setArmAuto(goal);
    }

    @Override
    public void end(boolean interrupted) {
       //elevatorSubsystem.stopElevator();
       aSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
