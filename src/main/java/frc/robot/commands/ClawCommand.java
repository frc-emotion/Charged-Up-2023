
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Boolean> clawFunc, clawStop;
    boolean direction, stopped; 


    private Timer timer;

    public ClawCommand(ClawSubsystem clawSubsystem, Supplier<Boolean> clawFunc, Supplier<Boolean> clawStop){
       
        this.clawSubsystem = clawSubsystem;
        this.clawFunc = clawFunc;
        this.clawStop = clawStop; 
        addRequirements(clawSubsystem);
        
        direction = false; 
        stopped = true; 

        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putBoolean("Stopped", stopped);
        SmartDashboard.putNumber("Claw Current Limit", 35);
        SmartDashboard.putNumber("Claw Forward Speed", 0.1);
    }


    public void execute(){
        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putBoolean("Stopped", stopped);

        SmartDashboard.putNumber("Time to Close", timer.get());

        //double clawCurrentLimit = SmartDashboard.getNumber("Claw Current Limit", 35);  
        //double clawSpeed = SmartDashboard.getNumber("Claw Forward Speed", 0.1);
         
        if (clawFunc.get()){
            timer.reset();
            direction = !direction; 
            //stopped = false; 
        } else if (clawStop.get()){
            timer.start();
        }

        if (timer.get() > 0.5){
            //stopped = true;
            clawSubsystem.stopClaw();
        } else if (direction && !stopped){
            clawSubsystem.setClawMotor(0.5);
        }
        else if (!direction && !stopped){
                clawSubsystem.setClawMotor(-0.25);
        }
        else if (stopped){
            clawSubsystem.stopClaw();
        }

        /* 
        if(direction && !stopped){
                clawSubsystem.setClawMotor(ClawConstants.clawNormalSpeed);
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
       clawSubsystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
