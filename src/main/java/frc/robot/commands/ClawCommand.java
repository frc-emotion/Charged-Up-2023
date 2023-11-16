
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
        // Check if game piece type should be switched
        if (leftBumper.get()) {
            clawSubsystem.switchGamePieceType(); // Switch it 
            gamePieceBoolean = !gamePieceBoolean; // Invert local variable

        }

        SmartDashboard.putBoolean("Game Piece Type", gamePieceBoolean); // I hope you know what this does
        
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

    @Override
    public void end(boolean interrupted) {
       clawSubsystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
