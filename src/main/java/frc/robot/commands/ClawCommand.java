
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Double> shouldIntake, shouldOutake; 
    boolean direction, stopped; 
    private int gamePieceType; // 0 for cube - 1 for cone 
    private Supplier<Boolean> leftBumper;
    private boolean gamePieceBoolean;

    //private Timer timer;

    public ClawCommand(ClawSubsystem clawSubsystem, int gamePieceType, Supplier<Double> shouldIntake, Supplier<Double> shouldOutake, Supplier<Boolean> leftBumper ){
       
        this.clawSubsystem = clawSubsystem;
        this.gamePieceType = gamePieceType;
        this.shouldIntake = shouldIntake; 
        this.shouldOutake = shouldOutake;
        this.leftBumper = leftBumper;
        if (gamePieceType == 0) { // Cube
            this.gamePieceBoolean = true;
        } else if (gamePieceType == 1) { // Cone
            this.gamePieceBoolean = false;
        }
        addRequirements(clawSubsystem);

        //timer = new Timer();
        
        // direction = false; 
        // stopped = true; 

        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putBoolean("Stopped", stopped);
        //SmartDashboard.putNumber("Game Piece Type (0-cube, 1-cone)", gamePieceType);
        SmartDashboard.putNumber("Claw Current Limit", 35);
        SmartDashboard.putNumber("Claw Forward Speed", 0.1);
    }

    @Override
    public void execute(){
        // SmartDashboard.putBoolean("Claw Direction", direction);
        // SmartDashboard.putBoolean("Stopped", stopped);

        //SmartDashboard.putNumber("Time to Close", timer.get());
        // System.out.println("hi");
        // System.out.println(leftBumper.get());

        //double clawCurrentLimit = SmartDashboard.getNumber("Claw Current Limit", 35);  
        //double clawSpeed = SmartDashboard.getNumber("Claw Forward Speed", 0.1);

        if (leftBumper.get()) {
            //System.out.println("Before: " + clawSubsystem.getPieceType());

            clawSubsystem.switchGamePieceType();
            gamePieceBoolean = !gamePieceBoolean;
            //clawSubsystem.setClawMotor(-0.3);

            //System.out.println("After: " + clawSubsystem.getPieceType());

        }

        SmartDashboard.putBoolean("Game Piece Type", gamePieceBoolean);

        

        

        // if (clawSubsystem.getPieceType() == 0) {
        //     System.out.println("Currently on cube");
        // } else if (clawSubsystem.getPieceType() == 1) {
        //     System.out.println("Currently on cone");
        // }
        

        switch (clawSubsystem.getPieceType()) {
            case 0: // Cube
                // Do routine for cube stuf
                if (shouldIntake.get() > Constants.ClawConstants.DEADZONE) {
                    // INTAKE
                    System.out.println("Attempting to set motor speed 1");
                    clawSubsystem.setClawMotor(Constants.ClawConstants.CUBE_INTAKE);
                } else if (shouldOutake.get() > Constants.ClawConstants.DEADZONE) {
                    // Outtake
                    System.out.println("Attempting to set motor speed 2");
                    clawSubsystem.setClawMotor(Constants.ClawConstants.CUBE_OUTTAKE);
                } else if (shouldOutake.get() < Constants.ClawConstants.DEADZONE) {
                    System.out.println("Stopping cube");
                    clawSubsystem.setClawMotor(0);
                }
                break;
                
            case 1: // Cone
                // Do routine for cone stuff
                if (shouldIntake.get() > Constants.ClawConstants.DEADZONE) {
                    // INTAKE 
                    System.out.println("Attempting to set motor speed 3");
                    clawSubsystem.setClawMotor(Constants.ClawConstants.CONE_INTAKE);
                } else if (shouldOutake.get() > Constants.ClawConstants.DEADZONE) {
                    // Outtake
                    System.out.println("Attempting to set motor speed 4");
                    clawSubsystem.setClawMotor(Constants.ClawConstants.CONE_OUTTAKE);
                } else if (shouldOutake.get() < Constants.ClawConstants.DEADZONE) {
                    System.out.println("Stopping cone");
                    clawSubsystem.setClawMotor(0);
                }
                break;
            }
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
