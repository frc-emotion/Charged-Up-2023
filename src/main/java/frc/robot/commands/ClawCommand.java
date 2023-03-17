package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Boolean> clawFunc;
    boolean direction;
    boolean stopped; 

    public ClawCommand(ClawSubsystem clawSubsystem, Supplier<Boolean> clawFunc){
        this.clawSubsystem = clawSubsystem;
        this.clawFunc = clawFunc;

        addRequirements(clawSubsystem);

        SmartDashboard.putNumber("Claw Current Limit", 35);
        direction = false; 
        stopped = true; 

    }


    public void execute(){

      /*   SmartDashboard.putBoolean("Claw Direction", direction);
        double clawCurrentLimit = SmartDashboard.getNumber("Claw Current Limit", 20);

        if (clawFunc.get()){
            direction = !direction; 
            stopped = false; 
        }

        if(direction && !stopped){
            clawSubsystem.setClawMotor(ClawConstants.clawNormalSpeed);
            if (Math.abs(clawSubsystem.getClawCurrent()) < clawCurrentLimit){
                clawSubsystem.setClawMotor(ClawConstants.clawNormalSpeed);
            }
            else if (Math.abs(clawSubsystem.getClawCurrent()) >= clawCurrentLimit){
                clawSubsystem.stopClaw();
                stopped = true; 
            }
        }
        else if(!direction && !stopped){
            if (Math.abs(clawSubsystem.getClawCurrent()) < clawCurrentLimit){
                clawSubsystem.setClawMotor(-ClawConstants.clawNormalSpeed);
            }
            else if (Math.abs(clawSubsystem.getClawCurrent()) >= clawCurrentLimit){
                clawSubsystem.stopClaw(); 
                stopped = true;
            }
        }
        */

    }

    @Override
    public void end(boolean interrupted) {
      // clawSubsystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}