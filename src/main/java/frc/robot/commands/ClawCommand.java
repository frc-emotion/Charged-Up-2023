
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase{
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Double> shouldIntake, shouldOutake; 
    boolean direction, stopped; 
    private int gamePieceType; // 0 for cube - 1 for cone 
    private boolean leftBumper;

    //private Timer timer;

    public ClawCommand(ClawSubsystem clawSubsystem, int gamePieceType, Supplier<Double> shouldIntake, Supplier<Double> shouldOutake, Boolean leftBumper ){
       
        this.clawSubsystem = clawSubsystem;
        this.gamePieceType = gamePieceType;
        this.shouldIntake = shouldIntake; 
        this.shouldOutake = shouldOutake;
        this.leftBumper = leftBumper;
        addRequirements(clawSubsystem);

        //timer = new Timer();
        
        // direction = false; 
        // stopped = true; 

        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putBoolean("Stopped", stopped);
        SmartDashboard.putNumber("Game Piece Type (0-cube, 1-cone)", gamePieceType);
        SmartDashboard.putNumber("Claw Current Limit", 35);
        SmartDashboard.putNumber("Claw Forward Speed", 0.1);
    }


    public void execute(){
        // SmartDashboard.putBoolean("Claw Direction", direction);
        // SmartDashboard.putBoolean("Stopped", stopped);

        //SmartDashboard.putNumber("Time to Close", timer.get());

        //double clawCurrentLimit = SmartDashboard.getNumber("Claw Current Limit", 35);  
        //double clawSpeed = SmartDashboard.getNumber("Claw Forward Speed", 0.1);
         
        if (leftBumper) {
            clawSubsystem.switchGamePieceType();
        }
        

        // switch (gamePieceType) {
        //     case 0: // Cube
        //         // Do routine for cube stuf
        //         if (shouldIntake.get() > 0.2) {
        //             // INTAKE
        //             System.out.println("Attempting to set motor speed 1");
        //             clawSubsystem.setClawMotor(-0.3);
        //         } else if (shouldOutake.get() > 0.2) {
        //             // Outtake
        //             System.out.println("Attempting to set motor speed 2");
        //             clawSubsystem.setClawMotor(0.3);
        //         }
        //         break;
        //     case 1: // Cone
        //         // Do routine for cone stuff
        //         if (shouldIntake.get() > 0.2) {
        //             // INTAKE 
        //             System.out.println("Attempting to set motor speed 3");
        //             clawSubsystem.setClawMotor(-0.5);
        //         } else if (shouldOutake.get() > 0.2) {
        //             // Outtake
        //             System.out.println("Attempting to set motor speed 4");
        //             clawSubsystem.setClawMotor(0.5);
        //         }
        //         break;
        }

        // if (clawFunc.get()){
        //     timer.reset();
        //     direction = !direction; 
        //     stopped = false; 
        // } else if (clawStop.get()){
        //     stopped = true;
        // }

        /*if (timer.get() > 0.5){
            //stopped = true;
            clawSubsystem.stopClaw();
        } else*/ 
        // if (direction && !stopped){
        //     clawSubsystem.setClawMotor(0.5);
        // }
        // else if (!direction && !stopped){
        //         clawSubsystem.setClawMotor(-0.25);
        // }
        // else if (stopped){
        //     clawSubsystem.stopClaw();
        // }

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
   // }

    @Override
    public void end(boolean interrupted) {
       clawSubsystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
