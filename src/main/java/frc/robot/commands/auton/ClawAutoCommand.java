
package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
