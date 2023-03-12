package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    
    private Intake intake;

    public RunIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        System.out.println("run");
        if(RobotContainer.operatorController.getLeftBumperPressed()){
            intake.intakeForward();
        }
        else if(RobotContainer.operatorController.getLeftBumperReleased()){
            intake.intakeStop();
        }

        if(RobotContainer.operatorController.getRightBumperPressed()){
            intake.intakeReverse();
        }
        else if(RobotContainer.operatorController.getRightBumperReleased()){
            intake.intakeStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
