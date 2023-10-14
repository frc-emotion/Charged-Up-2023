
package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawAutoCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;

    boolean dir;

    public ClawAutoCommand(ClawSubsystem clawSubsystem, boolean dir){
       
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
        this.dir = dir;
    }


    public void execute(){
        clawSubsystem.setClawMotor(dir ? 0.5 : -0.25);
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
