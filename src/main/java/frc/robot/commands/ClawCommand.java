package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        public void execute(){

        boolean direction = false; 

        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putNumber("Claw Current Limit", 35);

        double clawCurrentLimit = SmartDashboard.getNumber("Claw Current Limit", 20);

        if(clawFunc.get() && !direction){
            if (clawSubsystem.getClawCurrent() < clawCurrentLimit){
                clawSubsystem.setClawMotor(ClawConstants.clawNormalSpeed);
            }
            else if (clawSubsystem.getClawCurrent() >= clawCurrentLimit){
                clawSubsystem.stopClaw();
                direction = true; 
            }
        }
        else if(clawFunc.get() && direction){
            if (clawSubsystem.getClawCurrent() < clawCurrentLimit){
                clawSubsystem.setClawMotor(-ClawConstants.clawNormalSpeed);
            }
            else if (clawSubsystem.getClawCurrent() >= clawCurrentLimit){
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