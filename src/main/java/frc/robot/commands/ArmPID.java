package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPID extends CommandBase {

    private ArmSubsystem aSubsystem;

    private double goal;

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
