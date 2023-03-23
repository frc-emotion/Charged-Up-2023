package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Boolean> clawFuncL, clawFuncR;
    boolean direction;
    boolean stopped; 

    public ClawCommand(ClawSubsystem clawSubsystem, Supplier<Boolean> clawFuncL, Supplier<Boolean> clawFuncR){
        this.clawSubsystem = clawSubsystem;
        this.clawFuncL = clawFuncL;
        this.clawFuncR = clawFuncR;

        addRequirements(clawSubsystem);

        direction = false; 
        stopped = true; 

    }

    public void execute(){


        /*if (clawFuncL.get()){
            clawSubsystem.setClawMotor(-0.1);
        } else if (clawFuncR.get()){
            clawSubsystem.setClawMotor(0.1);
        } else {
            clawSubsystem.stopClaw();
        } */


    
        if (clawFuncR.get()){
            clawSubsystem.setClawMotor(0.2);
        } else if (clawFuncL.get()) {
            clawSubsystem.setClawMotor(-0.1);
        }

        /*if (direction && !stopped){
            clawSubsystem.setClawMotor(0.1);
        } else if (!direction && !stopped){
            clawSubsystem.setClawMotor(-0.1);
        }*/

        SmartDashboard.putNumber("claw current 2.0", clawSubsystem.signedOutputCurrent());
        SmartDashboard.putNumber("claww speed", clawSubsystem.getSpeed());
        



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