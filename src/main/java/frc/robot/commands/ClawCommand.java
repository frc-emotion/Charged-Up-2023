package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Boolean> clawFunc;

    public ClawCommand(ClawSubsystem clawSubsystem, Supplier<Boolean> clawFunc){
        this.clawSubsystem = clawSubsystem;
        this.clawFunc = clawFunc;

        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){

        boolean direction = false; 

        if(clawFunc.get() && !direction){
            if (clawSubsystem.getClawCurrent() < ClawConstants.closedClawCurrentThreshold){
                clawSubsystem.setClawMotor(ClawConstants.clawNormalSpeed);
            }
            else if (clawSubsystem.getClawCurrent() >= ClawConstants.closedClawCurrentThreshold){
                clawSubsystem.stopClaw();
                direction = true; 
            }
        }
        else if(clawFunc.get() && direction){
            if (clawSubsystem.getClawCurrent() < ClawConstants.closedClawCurrentThreshold){
                clawSubsystem.setClawMotor(-ClawConstants.clawNormalSpeed);
            }
            else if (clawSubsystem.getClawCurrent() >= ClawConstants.closedClawCurrentThreshold){
                clawSubsystem.stopClaw(); 
                direction = false; 
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
       clawSubsystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}