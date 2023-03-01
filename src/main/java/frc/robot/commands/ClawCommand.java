package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;

    public ClawCommand(ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }


    

}
